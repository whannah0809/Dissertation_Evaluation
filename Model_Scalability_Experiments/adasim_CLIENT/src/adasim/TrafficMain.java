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
 *Generated additional methods for testing the computation time of the digital
 *twin integrated system at different model amounts.
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
	private static ComputationTimes times = new ComputationTimes();

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
	public static SimulationResults trySimulation(ConfigurationOptions opts, BPA[] node_uncertainties, float[] sensor_accuracies, ComputationTimes times){
		final Duration timeout = Duration.ofMillis(1000);
		try{
			simulationSemaphore.acquire();
		} catch (InterruptedException e){};

		ExecutorService executor = Executors.newSingleThreadExecutor();
		try {
			final Future<SimulationResults> handler = executor.submit(new Callable<SimulationResults>() {
				@Override
				public SimulationResults call() throws Exception {
					try {
						TrafficSimulator tsim = SimulationXMLReader.buildSimulator( new File(opts.getInputFile() ), node_uncertainties, sensor_accuracies, cur_strat, times);
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
				times.AddTime(1000f);
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
	 * Method to output computation times of the route finding to a csv file
	 * @author William Hannah
	 */
	private static void OutputCSV(float[] times){
		String csvFile = "data.csv";

        try (FileWriter writer = new FileWriter(csvFile)) {
            for (float value : times) {
                writer.append(String.valueOf(value)).append('\n');
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

		BasicConfigurator.configure();
		ConfigurationOptions opts1 = new ConfigurationOptions("resources/test/config.xml");

		float[] sensor_accuracies1 = {5f, 4f, 6f, 4f, 5f, 8f, 9f, 5f, 6f, 6f};

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

		for(int r = 0; r < 100; r++){
			cur_strat = 0;

			BPA[] bpas = node_uncertainties.toArray(new BPA[0]);
			BPA[] bpasToSend = new BPA[bpas.length];
			for(int i = 0; i < bpas.length; i++){
				bpasToSend[i] = bpas[i].generateSendableBPA();
			}
			SimulationResults result = trySimulation(opts1, bpas, sensor_accuracies1, times);
			updateBPAs(result.path_taken, result.delay_observations);
			ConnectToDigitalTwin(1, bpasToSend);
			ConnectToDigitalTwin(4, result.correct_path);
			try {
				Thread.sleep(200); 
			} catch (InterruptedException ex) {
				ex.printStackTrace();
			}
		}

		List<Float> timesList = times.GetComputationTimes();
		float[] array_times = new float[timesList.size()];
		for (int i = 0; i < timesList.size(); i++) {
			array_times[i] = timesList.get(i);
		}
		OutputCSV(array_times);		
	}
}
