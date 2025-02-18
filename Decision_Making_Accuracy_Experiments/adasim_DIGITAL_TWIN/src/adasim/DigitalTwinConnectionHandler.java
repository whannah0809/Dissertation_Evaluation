package adasim;

import org.apache.log4j.Logger;
import org.apache.log4j.BasicConfigurator;
import org.jdom.JDOMException;

import java.io.*;
import java.net.*;
import java.util.concurrent.*;

import adasim.uncertainty_handling.BPA;


public class DigitalTwinConnectionHandler extends Thread {
    private final BlockingDeque<float[]> strategy_queue;
    private final BlockingDeque<float[]> sensor_acc_queue;
    private final BlockingDeque<BPA[]> uncertainty_queue;
    private final BlockingDeque<ConfigurationOptions> config_queue;
    private final BlockingDeque<Boolean> feedback_queue;
    private static final float[] preference_adjuster = {10f, 10f, 10f, 10f};
    private final Semaphore semaphore;

    public DigitalTwinConnectionHandler(Semaphore semaphore, BlockingDeque<float[]> strategy_queue, BlockingDeque<BPA[]> uncertainty_queue, BlockingDeque<ConfigurationOptions> config_queue, BlockingDeque<float[]> sensor_acc_queue, BlockingDeque<Boolean> feedback_queue) {
        this.strategy_queue = strategy_queue;
        this.uncertainty_queue = uncertainty_queue;
        this.config_queue = config_queue;
        this.sensor_acc_queue = sensor_acc_queue;
        this.feedback_queue = feedback_queue;
        this.semaphore = semaphore;

        try{
            float[] init = {1f, 0f, 0f, 0f};
            strategy_queue.putFirst(init);
        }
        catch(InterruptedException e){
            e.printStackTrace();
        }
    }

    public static void adjustPreference(int strategy, boolean feedback){
		if (feedback) {
			preference_adjuster[strategy] = (preference_adjuster[strategy] == 10) ? 10 : preference_adjuster[strategy] + 1;
		} else {
			preference_adjuster[strategy] = (preference_adjuster[strategy] == 1) ? 1 : preference_adjuster[strategy] - 1;
		}
	}

    private void updateConfig(ObjectInputStream inputStream){
        try{
            Object obj = inputStream.readObject();
            if (obj instanceof String) {
                String xmlContent = (String) obj;
                
                // Write XML content to file
                try (PrintWriter fileWriter = new PrintWriter(new FileWriter("resources/output.xml"))) {
                    fileWriter.println(xmlContent);
                }
            } else {
                System.err.println("Failed to recieve config file");
            }

            ConfigurationOptions config = new ConfigurationOptions("resources/output.xml");
            
            semaphore.acquire();
            config_queue.putFirst(config);
        }
        catch(IOException | ClassNotFoundException | InterruptedException e){
            e.printStackTrace();
        }
    }

    private void updateBPA(ObjectInputStream inputStream){
        try{
            BPA[] receivedBPAArray = (BPA[]) inputStream.readObject();
            uncertainty_queue.putFirst(receivedBPAArray);

            semaphore.release();
        }
        catch(IOException | ClassNotFoundException | InterruptedException e){
            e.printStackTrace();
        }
    }

    private void updateSensorAccuracy(ObjectInputStream inputStream){
        try{
            float[] recieved_accuracies = (float[]) inputStream.readObject();
            sensor_acc_queue.clear();
            sensor_acc_queue.put(recieved_accuracies);
        }
        catch(IOException | ClassNotFoundException | InterruptedException e){
            e.printStackTrace();
        }
    }

    private void sendStrategy(DataOutputStream outputStream){
        float[] cur_pref = strategy_queue.peek();
        try{
            int best_strat = 0;
            for(int i = 0; i < strategy_queue.peek().length; i++){
                if(cur_pref[i] * preference_adjuster[i] > cur_pref[best_strat] *preference_adjuster[best_strat]){
                    best_strat = i;
                }
            }
            int data = best_strat; // Get the latest value without removing it from the queue
            outputStream.writeInt(data);
        }catch(IOException e){
            e.printStackTrace();
        }
    }   

    private void recieveFeedback(ObjectInputStream inputStream){
        try{
            boolean recieved_feedback = (boolean) inputStream.readObject();
            int used_strategy = (int) inputStream.readObject();
            adjustPreference(used_strategy, recieved_feedback);
        }catch(IOException | ClassNotFoundException e){
            e.printStackTrace();
        }
    }

    public void run() {
        while (true) {
            try (ServerSocket serverSocket = new ServerSocket(8888)) {
                Socket clientSocket = serverSocket.accept();

                try (ObjectInputStream inputStream = new ObjectInputStream(clientSocket.getInputStream())) {
                    
                    int control = -1;

                    // Receive data from the client
                    while(control == -1){
                        control = inputStream.readInt(); //0 for updating config, 1 for updating BPA, 2 for feedback.
                    }

                    switch(control){
                        case 0:
                            updateConfig(inputStream);
                            break;
                        case 1:
                            updateBPA(inputStream);
                            break;
                        case 2:
                            try(DataOutputStream outputStream = new DataOutputStream(clientSocket.getOutputStream())){
                                sendStrategy(outputStream);
                            }
                            break;
                        case 3:
                            updateSensorAccuracy(inputStream);
                            break;
                        case 4:
                            recieveFeedback(inputStream);
                            break;
                        default:
                    }

                    try{
                        Thread.sleep(100);
                    } catch (InterruptedException e) {}

                } catch (IOException e) {
                    // Handle exceptions related to reading from or writing data to the client
                    e.printStackTrace();
                }
            
            } catch (IOException e) {
                // Handle exceptions related to accepting client connections
                e.printStackTrace();
            }
        }
    }
}
