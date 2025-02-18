/*******************************************************************************
 * Copyright (C) 2011 - 2012 Jochen Wuttke, Jonathan Ramaswamy
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is furnished to do
 * so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * Contributors:
 *    Jonathan Ramaswamy (ramaswamyj12@gmail.com) - initial API and implementation
 ********************************************************************************
 *
 * Created: Jul 29, 2011
 */

package adasim;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;

import java.io.*;
import java.net.*;
import java.util.*;
import java.util.concurrent.*;
import java.time.*;

import org.apache.log4j.Logger;
import org.apache.log4j.BasicConfigurator;
import org.apache.commons.statistics.distribution.*;
import org.jdom.JDOMException;

import adasim.model.ConfigurationException;
import adasim.model.RoadSegment;
import adasim.model.TrafficSimulator;
import adasim.model.internal.SimulationXMLReader;
import adasim.uncertainty_handling.*;

/*******************************************************************************
 *Extended for Lancaster University Third Year Project.
 *Generated additional methods for testing the decision making accuracy of
 *various implementations
 ********************************************************************************
 *
 * William Hannah
 * Submitted: Mar 21, 2024
 */

/**
 * This is the main class for running the simulator.
 * <p>
 * It takes on commandline argument <code>-I file</code> with the name
 * of a configuration file defining a simulation. It will then load,
 * validate and run the simulation. All simulation output is 
 * written to stdout.
 * 
 * @author Jonathan Ramaswamy - ramaswamyj12@gmail.com
 */

public class TrafficMain {
	
	private static Logger logger = Logger.getLogger(TrafficMain.class); //Logger that outputs simulation information
	private static List<BPA> node_uncertainties = new ArrayList<>();
	private static int cur_strat;
	private static final Semaphore simulationSemaphore = new Semaphore(1);
	private static boolean dtConn = false;

	/**
	 * Method to initialize the BPA for testing.
	 * 
	 * @author William Hannah
	 */
	private static void initializeBPA(){
		//Assigning test values: 
		node_uncertainties.add(new BPA(
			new float[] {0.05f, 0.08f, 0.12f, 0.15f, 0.18f, 0.18f, 0.12f, 0.08f, 0.03f, 0.01f}, //Will be used as the real distribution for the simulation. Initialize with a prediction
			new float[][] {
				{0.06f, 0.09f, 0.13f, 0.16f, 0.20f, 0.17f, 0.10f, 0.07f, 0.02f, 0.01f},
				{0.07f, 0.10f, 0.14f, 0.17f, 0.21f, 0.16f, 0.11f, 0.08f, 0.03f, 0.01f},
				{0.05f, 0.08f, 0.12f, 0.15f, 0.19f, 0.18f, 0.13f, 0.09f, 0.04f, 0.02f}
			}
		));
		node_uncertainties.add(new BPA(
			new float[] {0.05f, 0.08f, 0.12f, 0.15f, 0.18f, 0.20f, 0.15f, 0.10f, 0.05f, 0.02f},
			new float[][] {
				{0.06f, 0.09f, 0.13f, 0.16f, 0.20f, 0.17f, 0.10f, 0.07f, 0.02f, 0.01f}, 
				{0.07f, 0.10f, 0.14f, 0.17f, 0.21f, 0.16f, 0.11f, 0.08f, 0.03f, 0.01f}, 
				{0.05f, 0.08f, 0.12f, 0.15f, 0.19f, 0.18f, 0.13f, 0.09f, 0.04f, 0.02f} 
			}

		));
		node_uncertainties.add(new BPA(
			new float[] {0.01f, 0.03f, 0.08f, 0.15f, 0.25f, 0.30f, 0.15f, 0.02f, 0.01f, 0.00f},
			new float[][] {
				{0.02f, 0.04f, 0.10f, 0.14f, 0.22f, 0.28f, 0.18f, 0.05f, 0.02f, 0.01f}, 
				{0.01f, 0.05f, 0.09f, 0.16f, 0.24f, 0.29f, 0.16f, 0.04f, 0.01f, 0.00f}, 
				{0.02f, 0.03f, 0.08f, 0.13f, 0.23f, 0.30f, 0.16f, 0.04f, 0.01f, 0.00f}
			}
		));
		
		node_uncertainties.add(new BPA(
			new float[]{0.20f, 0.15f, 0.10f, 0.08f, 0.06f, 0.05f, 0.04f, 0.03f, 0.02f, 0.01f},
			new float[][] {
				{0.22f, 0.16f, 0.11f, 0.09f, 0.07f, 0.06f, 0.05f, 0.04f, 0.03f, 0.02f},
				{0.18f, 0.14f, 0.09f, 0.07f, 0.05f, 0.04f, 0.03f, 0.02f, 0.01f, 0.00f},
				{0.21f, 0.16f, 0.11f, 0.08f, 0.06f, 0.05f, 0.04f, 0.03f, 0.02f, 0.01f}
			}
			));

		node_uncertainties.add(new BPA(
			new float[]{0.03f, 0.05f, 0.08f, 0.12f, 0.15f, 0.15f, 0.12f, 0.08f, 0.05f, 0.03f},
			new float[][] {
				{0.04f, 0.06f, 0.09f, 0.13f, 0.16f, 0.16f, 0.13f, 0.09f, 0.06f, 0.04f},
				{0.03f, 0.04f, 0.07f, 0.11f, 0.14f, 0.14f, 0.11f, 0.07f, 0.04f, 0.03f},
				{0.05f, 0.07f, 0.10f, 0.13f, 0.16f, 0.15f, 0.12f, 0.09f, 0.06f, 0.04f}
			}
		));

		node_uncertainties.add(new BPA(
			new float[]{0.05f, 0.08f, 0.12f, 0.18f, 0.20f, 0.15f, 0.10f, 0.08f, 0.03f, 0.01f},
			new float[][] {
				{0.06f, 0.09f, 0.13f, 0.17f, 0.19f, 0.14f, 0.09f, 0.07f, 0.02f, 0.01f},
				{0.04f, 0.07f, 0.11f, 0.17f, 0.21f, 0.16f, 0.11f, 0.09f, 0.04f, 0.01f},
				{0.05f, 0.08f, 0.12f, 0.18f, 0.20f, 0.15f, 0.10f, 0.07f, 0.02f, 0.01f}
			}	
		));

		node_uncertainties.add(new BPA(
			new float[]{0.01f, 0.02f, 0.05f, 0.10f, 0.20f, 0.25f, 0.20f, 0.12f, 0.04f, 0.01f},
			new float[][] {
				{0.02f, 0.03f, 0.06f, 0.11f, 0.21f, 0.26f, 0.21f, 0.13f, 0.05f, 0.02f},
				{0.01f, 0.02f, 0.05f, 0.09f, 0.19f, 0.24f, 0.19f, 0.11f, 0.03f, 0.01f},
				{0.01f, 0.02f, 0.05f, 0.10f, 0.20f, 0.25f, 0.20f, 0.12f, 0.04f, 0.01f}
			}
		));

		node_uncertainties.add(new BPA(
			new float[]{0.01f, 0.02f, 0.05f, 0.08f, 0.15f, 0.30f, 0.25f, 0.12f, 0.02f, 0.00f},
			new float[][]{
				{0.02f, 0.012f, 0.025f, 0.082f, 0.115f, 0.34f, 0.25f, 0.14f, 0.024f, 0.00f},
				{0.013f, 0.022f, 0.015f, 0.085f, 0.145f, 0.32f, 0.27f, 0.12f, 0.023f, 0.00f},
				{0.014f, 0.012f, 0.015f, 0.089f, 0.125f, 0.32f, 0.28f, 0.11f, 0.022f, 0.00f}
			}
		));

		node_uncertainties.add(new BPA(
			new float[]{0.10f, 0.08f, 0.06f, 0.05f, 0.04f, 0.03f, 0.02f, 0.01f, 0.01f, 0.00f},
			new float[][] {
				{0.02f, 0.03f, 0.06f, 0.09f, 0.14f, 0.29f, 0.24f, 0.11f, 0.03f, 0.00f},
				{0.01f, 0.02f, 0.05f, 0.08f, 0.16f, 0.31f, 0.26f, 0.13f, 0.03f, 0.00f},
				{0.01f, 0.02f, 0.05f, 0.08f, 0.15f, 0.30f, 0.25f, 0.12f, 0.02f, 0.00f}
			}
		));

		node_uncertainties.add(new BPA(
			new float[]{0.10f, 0.08f, 0.06f, 0.05f, 0.04f, 0.03f, 0.02f, 0.01f, 0.01f, 0.00f},
			new float[][] {
				{0.11f, 0.09f, 0.07f, 0.06f, 0.05f, 0.04f, 0.03f, 0.02f, 0.01f, 0.00f},
				{0.09f, 0.07f, 0.05f, 0.04f, 0.03f, 0.02f, 0.01f, 0.01f, 0.02f, 0.00f},
				{0.10f, 0.08f, 0.06f, 0.05f, 0.04f, 0.03f, 0.02f, 0.01f, 0.01f, 0.00f}
			}
		));
	}

	/**
	 * Method to add additional BPAs when map size increases
	 * 
	 * @author William Hannah
	 */
	private static void ExtendBPAs(){
		node_uncertainties.add(new BPA(
			new float[] {0.05f, 0.08f, 0.12f, 0.15f, 0.18f, 0.18f, 0.12f, 0.08f, 0.03f, 0.01f}, //Will be used as the real distribution for the simulation. Initialize with a prediction
			new float[][] {
				{0.06f, 0.09f, 0.13f, 0.16f, 0.20f, 0.17f, 0.10f, 0.07f, 0.02f, 0.01f},
				{0.07f, 0.10f, 0.14f, 0.17f, 0.21f, 0.16f, 0.11f, 0.08f, 0.03f, 0.01f},
				{0.05f, 0.08f, 0.12f, 0.15f, 0.19f, 0.18f, 0.13f, 0.09f, 0.04f, 0.02f}
			}
		));
		node_uncertainties.add(new BPA(
			new float[] {0.05f, 0.08f, 0.12f, 0.15f, 0.18f, 0.20f, 0.15f, 0.10f, 0.05f, 0.02f},
			new float[][] {
				{0.06f, 0.09f, 0.13f, 0.16f, 0.20f, 0.17f, 0.10f, 0.07f, 0.02f, 0.01f}, 
				{0.07f, 0.10f, 0.14f, 0.17f, 0.21f, 0.16f, 0.11f, 0.08f, 0.03f, 0.01f}, 
				{0.05f, 0.08f, 0.12f, 0.15f, 0.19f, 0.18f, 0.13f, 0.09f, 0.04f, 0.02f} 
			}

		));
		node_uncertainties.add(new BPA(
			new float[] {0.01f, 0.03f, 0.08f, 0.15f, 0.25f, 0.30f, 0.15f, 0.02f, 0.01f, 0.00f},
			new float[][] {
				{0.02f, 0.04f, 0.10f, 0.14f, 0.22f, 0.28f, 0.18f, 0.05f, 0.02f, 0.01f}, 
				{0.01f, 0.05f, 0.09f, 0.16f, 0.24f, 0.29f, 0.16f, 0.04f, 0.01f, 0.00f}, 
				{0.02f, 0.03f, 0.08f, 0.13f, 0.23f, 0.30f, 0.16f, 0.04f, 0.01f, 0.00f}
			}
		));
		
		node_uncertainties.add(new BPA(
			new float[]{0.20f, 0.15f, 0.10f, 0.08f, 0.06f, 0.05f, 0.04f, 0.03f, 0.02f, 0.01f},
			new float[][] {
				{0.22f, 0.16f, 0.11f, 0.09f, 0.07f, 0.06f, 0.05f, 0.04f, 0.03f, 0.02f},
				{0.18f, 0.14f, 0.09f, 0.07f, 0.05f, 0.04f, 0.03f, 0.02f, 0.01f, 0.00f},
				{0.21f, 0.16f, 0.11f, 0.08f, 0.06f, 0.05f, 0.04f, 0.03f, 0.02f, 0.01f}
			}
			));

		node_uncertainties.add(new BPA(
			new float[]{0.03f, 0.05f, 0.08f, 0.12f, 0.15f, 0.15f, 0.12f, 0.08f, 0.05f, 0.03f},
			new float[][] {
				{0.04f, 0.06f, 0.09f, 0.13f, 0.16f, 0.16f, 0.13f, 0.09f, 0.06f, 0.04f},
				{0.03f, 0.04f, 0.07f, 0.11f, 0.14f, 0.14f, 0.11f, 0.07f, 0.04f, 0.03f},
				{0.05f, 0.07f, 0.10f, 0.13f, 0.16f, 0.15f, 0.12f, 0.09f, 0.06f, 0.04f}
			}
		));

		node_uncertainties.add(new BPA(
			new float[]{0.05f, 0.08f, 0.12f, 0.18f, 0.20f, 0.15f, 0.10f, 0.08f, 0.03f, 0.01f},
			new float[][] {
				{0.06f, 0.09f, 0.13f, 0.17f, 0.19f, 0.14f, 0.09f, 0.07f, 0.02f, 0.01f},
				{0.04f, 0.07f, 0.11f, 0.17f, 0.21f, 0.16f, 0.11f, 0.09f, 0.04f, 0.01f},
				{0.05f, 0.08f, 0.12f, 0.18f, 0.20f, 0.15f, 0.10f, 0.07f, 0.02f, 0.01f}
			}	
		));
	}

	/**
	 * Updates the BPAs (Basic Probability Assignments) based on the path taken and delay observations.
	 * 
	 * @param path_taken The path taken during the simulation.
	 * @param delay_observations The delay observations made during the simulation.
	 * 
	 * @author William Hannah
	 */
	public static void updateBPAs(List<Integer> path_taken, List<Integer> delay_observations){
		if(path_taken != null){
			for(int i = 0; i < path_taken.size(); i++){
				int observation = (int) delay_observations.get(i)/10;
				node_uncertainties.get(path_taken.get(i)).updateDistributions(observation);
			}
		}
	}

	/**
	 * Attempts a simulation based on the provided configuration options, node uncertainties, sensor accuracies, and computation times.
	 * 
	 * @param opts The configuration options for the simulation.
	 * @param node_uncertainties The uncertainties associated with each node in the simulation.
	 * @param sensor_accuracies The accuracies of the sensors used in the simulation.
	 * 
	 * @return The results of the simulation.
	 * 
	 * @author William Hannah
	 */
	public static SimulationResults trySimulation(ConfigurationOptions opts, BPA[] node_uncertainties, float[] sensor_accuracies){
		final Duration timeout = Duration.ofMillis(500);
		try{
			simulationSemaphore.acquire();
		} catch (InterruptedException e){};

		ExecutorService executor = Executors.newSingleThreadExecutor();
		try {
			final Future<SimulationResults> handler = executor.submit(new Callable<SimulationResults>() {
				@Override
				public SimulationResults call() throws Exception {
					try {
						TrafficSimulator tsim = SimulationXMLReader.buildSimulator( new File(opts.getInputFile() ), node_uncertainties, sensor_accuracies, cur_strat);
						SimulationResults simresult = tsim.run();
						return simresult;
					} finally {
						simulationSemaphore.release();
					}
				}
			});

			SimulationResults result = new SimulationResults(null, false, null);
			try {
				result = handler.get(timeout.toMillis(), TimeUnit.MILLISECONDS);
			} catch (TimeoutException e) {
				handler.cancel(true);
			} catch (InterruptedException | ExecutionException e) {
				e.printStackTrace();
			}

			return result;
		} finally {
			executor.shutdownNow();
		}
	}

	/**
	 * Method to establish a connection with the digital twin to send data
	 * 
	 * @param control The protocol for recieving and sending data
	 * @param object the object to send
	 * 
	 * @author William Hannah
	 */
	private static void ConnectToDigitalTwin(int control, Object toSend) {
		logger.info("ATTEMPTING DT CONNECTION");
		boolean sent = false;

		while (!sent) {
			try (Socket socket = new Socket("localhost", 8888);
				ObjectOutputStream objectOutputStream = new ObjectOutputStream(socket.getOutputStream())) {

				// Write control and object to the ObjectOutputStream
				objectOutputStream.writeInt(control);
				objectOutputStream.writeObject(toSend);
				if(control == 4){
					objectOutputStream.writeObject(cur_strat);
				}
				objectOutputStream.flush();
				sent = true;

				socket.close();
			} catch (IOException e) {
			}

			try {
				Thread.sleep(100); 
			} catch (InterruptedException ex) {
				ex.printStackTrace();
			}
		}
	}

	/**
	 * Method to establish a connection with the digital twin to recieve the current best strategy
	 * 
	 * @author William Hannah
	 */
	private static final int GetStrategy() {
		boolean connected = false;
		int control = 2;
		int strategy = -1;
		while (!connected) {
			try (Socket socket = new Socket("localhost", 8888);
				ObjectOutputStream outputStream = new ObjectOutputStream(socket.getOutputStream());
				DataInputStream inputStream = new DataInputStream(socket.getInputStream())) {

				outputStream.writeInt(control);
				outputStream.flush();

				strategy = inputStream.readInt();
				
				connected = true; // Connection successful, set connected flag to true

			} catch (IOException e) {
				try {
					Thread.sleep(100); // Wait for 1 second before retrying
				} catch (InterruptedException ex) {
					ex.printStackTrace();
				}
			}
		}
		return strategy;
	}

	/**
	 * Method to output decision making accuracies to a csv file
	 * @author William Hannah
	 */
	private static void OutputCSV(boolean[] success, int[] strats){
		String csvFile = "data.csv";

        try (FileWriter writer = new FileWriter(csvFile)) {
            for (boolean value : success) {
                writer.append(String.valueOf(value)).append('\n');
            }
			for (int s_value : strats) {
                writer.append(String.valueOf(s_value)).append('\n');
            }
            System.out.println("Data has been written to " + csvFile);
        } catch (IOException e) {
            System.err.println("Error writing to CSV file: " + e.getMessage());
            e.printStackTrace();
        }
	}

	
	/**
	 * 
	 * @param args
	 * @throws JDOMException
	 * @throws IOException
	 * @throws ConfigurationException
	 * 
	 * extended by William Hannah to test decision making accuracy of various implementations
	 */
	public static void main(String[] args) {
		if (args.length == 0) {
            System.out.println("No command-line arguments provided.");
        } else {
            cur_strat =  Integer.parseInt(args[0]);
			if(cur_strat == 4){
				dtConn = true;
			}
		}

		String[] strategies = {"Delay Estimation - Point Estimate", "Delay Estimation - Interval Sample = 1", "Delay Quereying - Point Estimate", "Delay Quereying - Interval Sample = 1"};
		BasicConfigurator.configure();
		ConfigurationOptions opts1 = new ConfigurationOptions("resources/test/config.xml");
		ConfigurationOptions opts2 = new ConfigurationOptions("resources/test/config_large_map.xml");

		float[] sensor_accuracies1 = {5f, 4f, 6f, 4f, 5f, 8f, 9f, 5f, 6f, 6f};
		float[] sensor_accuracies2 = {32f, 31f, 29f, 26f, 34f, 42f, 35f, 35f, 40f, 31f};
		float[] sensor_accuracies3 = {32f, 31f, 29f, 26f, 34f, 42f, 35f, 35f, 40f, 31f, 32f, 28f, 31f, 37f, 35f, 38f};

		boolean[] succession = new boolean[150];
		int[] strategies_used = new int[150];

		//------------------------------------------SCENARIO 1------------------------------------------

		String filePath1 = opts1.getInputFile();
        StringBuilder xmlContentBuilder = new StringBuilder();
        try (BufferedReader fileReader = new BufferedReader(new FileReader(filePath1))) {
            String line;
            while ((line = fileReader.readLine()) != null) {
                xmlContentBuilder.append(line).append("\n");
            }
        } catch (IOException e) {
            e.printStackTrace();
            return;
        }

		initializeBPA();
		BPA[] uncertainty_array = node_uncertainties.toArray(new BPA[0]);
        String xmlContent = xmlContentBuilder.toString();

		ConnectToDigitalTwin(0, xmlContent);
		try {
			Thread.sleep(100); 
		} catch (InterruptedException ex) {
			ex.printStackTrace();
		}
		ConnectToDigitalTwin(1, uncertainty_array);
		try {
			Thread.sleep(100); 
		} catch (InterruptedException ex) {
			ex.printStackTrace();
		}
		ConnectToDigitalTwin(3, sensor_accuracies1);

		for(int r = 0; r < 50; r++){
			if(dtConn){
				cur_strat = GetStrategy();
			}
			BPA[] bpas = node_uncertainties.toArray(new BPA[0]);
			BPA[] bpasToSend = new BPA[bpas.length];
			for(int i = 0; i < bpas.length; i++){
				bpasToSend[i] = bpas[i].generateSendableBPA();
			}
			SimulationResults result = trySimulation(opts1, bpas, sensor_accuracies1);
			updateBPAs(result.path_taken, result.delay_observations);
			ConnectToDigitalTwin(1, bpasToSend);
			succession[r] = result.correct_path;
			strategies_used[r] = cur_strat; 
			ConnectToDigitalTwin(4, result.correct_path);
			//strategies_used[r] = 
			try {
				Thread.sleep(100); 
			} catch (InterruptedException ex) {
				ex.printStackTrace();
			}
		}


		//------------------------------------------SCENARIO 2------------------------------------------

		ConnectToDigitalTwin(3, sensor_accuracies2);
		for(int r = 0; r < 50; r++){
			if(dtConn){
				cur_strat = GetStrategy();
			}			BPA[] bpas = node_uncertainties.toArray(new BPA[0]);
			BPA[] bpasToSend = new BPA[bpas.length];
			for(int i = 0; i < bpas.length; i++){
				bpasToSend[i] = bpas[i].generateSendableBPA();
			}
			SimulationResults result = trySimulation(opts1, bpas, sensor_accuracies2);
			updateBPAs(result.path_taken, result.delay_observations);
			ConnectToDigitalTwin(1, bpasToSend);
			succession[50 + r] = result.correct_path;
			strategies_used[50 + r] = cur_strat; 
			ConnectToDigitalTwin(4, result.correct_path);
			try {
				Thread.sleep(100); 
			} catch (InterruptedException ex) {
				ex.printStackTrace();
			}
		}

		//------------------------------------------SCENARIO 3------------------------------------------

		String filePath2 = opts2.getInputFile();
        StringBuilder xmlContentBuilder_lm = new StringBuilder();
        try (BufferedReader fileReader_lm = new BufferedReader(new FileReader(filePath2))) {
            String line;
            while ((line = fileReader_lm.readLine()) != null) {
                xmlContentBuilder_lm.append(line).append("\n");
            }
        } catch (IOException e) {
            e.printStackTrace();
            return;
        }
		ExtendBPAs();
		BPA[] uncertainty_array_lm = node_uncertainties.toArray(new BPA[0]);
        String xmlContent_lm = xmlContentBuilder_lm.toString();

		ConnectToDigitalTwin(0, xmlContent_lm);
		try {
			Thread.sleep(100); 
		} catch (InterruptedException ex) {
			ex.printStackTrace();
		}
		ConnectToDigitalTwin(1, uncertainty_array_lm);
		try {
			Thread.sleep(100); 
		} catch (InterruptedException ex) {
			ex.printStackTrace();
		}
		ConnectToDigitalTwin(3, sensor_accuracies3);

		for(int r = 0; r < 50; r++){
			if(dtConn){
				cur_strat = GetStrategy();
			}
			BPA[] bpas = node_uncertainties.toArray(new BPA[0]);
			BPA[] bpasToSend = new BPA[bpas.length];
			for(int i = 0; i < bpas.length; i++){
				bpasToSend[i] = bpas[i].generateSendableBPA();
			}
			SimulationResults result = trySimulation(opts2, bpas, sensor_accuracies3);
			updateBPAs(result.path_taken, result.delay_observations);
			ConnectToDigitalTwin(1, bpasToSend);
			succession[r + 100] = result.correct_path;
			strategies_used[r + 100] = cur_strat; 
			ConnectToDigitalTwin(4, result.correct_path);
			try {
				Thread.sleep(100); 
			} catch (InterruptedException ex) {
				ex.printStackTrace();
			}
		}

		OutputCSV(succession, strategies_used);
	}
}
