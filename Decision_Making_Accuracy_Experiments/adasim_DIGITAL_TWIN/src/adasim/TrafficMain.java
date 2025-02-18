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
import adasim.uncertainty_handling.BPA;

/*******************************************************************************
 *Extended for Lancaster University Third Year Project.
 *Generated additional methods for testing the decision making accuracy of
 *a digital twin integrated implementation.
 ********************************************************************************
 *
 * William Hannah
 * 
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

    private static final BlockingDeque<float[]> strategy_queue = new LinkedBlockingDeque<>();
    private static final BlockingDeque<BPA[]> uncertainty_queue = new LinkedBlockingDeque<>();
	private static final BlockingDeque<ConfigurationOptions> config_queue = new LinkedBlockingDeque<>();
	private static final BlockingDeque<float[]> sensor_acc_queue = new LinkedBlockingDeque<>();
	private static final Semaphore semaphore = new Semaphore(1);
	private static final Semaphore simulationSemaphore = new Semaphore(1);
	private static final BlockingDeque<Boolean> feedback_queue = new LinkedBlockingDeque<>();


	/**
	 * Performs a Bayesian update on a Beta distribution.
	 * 
	 * This method updates the parameters of a Beta distribution based on observed successes and failures
	 * 
	 * @param n The total number of trials.
	 * @param k The number of successful trials.
	 * @param beta_prior The prior Beta distribution.
	 * 
	 * @return The posterior Beta distribution after the Bayesian update.
	 * 
	 * @author William Hannah
	 */
	public static BetaDistribution bayesianUpdateBeta(int n, int k, BetaDistribution beta_prior) {
		double alpha_posterior_val = beta_prior.getAlpha() + k; 
		double beta_posterior_val = beta_prior.getBeta() + n - k;

        BetaDistribution beta_posterior = BetaDistribution.of(alpha_posterior_val, beta_posterior_val);

        return beta_posterior;
    }

	/**
	 * Attempts a simulation with a specified strategy and configuration options.
	 * 
	 * This method tries to run a simulation using the specified strategy, configuration options, node uncertainties,
	 * and sensor accuracies, and returns whether the simulation was successful or not.
	 * 
	 * @param strategy The strategy to be used in the simulation.
	 * @param opts The configuration options for the simulation.
	 * @param node_uncertainties The uncertainties associated with each node in the simulation.
	 * @param sensor_accuracies The accuracies of the sensors used in the simulation.
	 * 
	 * @return true if the tested strategy succcessfully routed the agent along the correct path
	 * 
	 * @author William Hannah
	 */
	public static boolean trySimulation(int strategy, ConfigurationOptions opts, BPA[] node_uncertainties, float[] sensor_accuracies) {
		final Duration timeout = Duration.ofMillis(500);
		try{
			simulationSemaphore.acquire();
		} catch (InterruptedException e){};

		ExecutorService executor = Executors.newSingleThreadExecutor();
		try {
			final Future<Boolean> handler = executor.submit(new Callable<Boolean>() {
				@Override
				public Boolean call() throws Exception {
					try {
						TrafficSimulator tsim = SimulationXMLReader.buildSimulator(new File(opts.getInputFile()), node_uncertainties, strategy, sensor_accuracies);
						boolean result = tsim.run();
						return result;
					} finally {
						simulationSemaphore.release();
					}
				}
			});

			boolean result = false;
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
	 * 
	 * @param args
	 * @throws JDOMException
	 * @throws IOException
	 * @throws ConfigurationException
	 * 
	 * extended by William Hannah to test decision making accuracy of digital twin integrated system
	 */
	public static void main(String[] args) {
		BasicConfigurator.configure();
		logger.info( Version.versionString() );
		
		String[] strategies = {"Delay Estimation - Point Estimate", "Delay Estimation - Interval Sample = 1", "Delay Quereying - Point Estimate", "Delay Quereying - Interval Sample = 1"};
		boolean[] result_list = new boolean[strategies.length];
		float[] strategy_beliefs = {0.4f, 0.3f, 0.6f, 0.7f};
		
		BetaDistribution strategy_0_beta = BetaDistribution.of(1 + strategy_beliefs[0], 1 - strategy_beliefs[0]);
		BetaDistribution strategy_1_beta = BetaDistribution.of(1 + strategy_beliefs[1], 1 - strategy_beliefs[1]);
		BetaDistribution strategy_2_beta = BetaDistribution.of(1 + strategy_beliefs[2], 1 - strategy_beliefs[2]);
		BetaDistribution strategy_3_beta = BetaDistribution.of(1 + strategy_beliefs[3], 1 - strategy_beliefs[3]);

		BetaDistribution strategy_betas[] = {strategy_0_beta, strategy_1_beta, strategy_2_beta, strategy_3_beta};

		int[] totals = {0, 0, 0, 0};

		DigitalTwinConnectionHandler connectionHandler = new DigitalTwinConnectionHandler(semaphore, strategy_queue, uncertainty_queue, config_queue, sensor_acc_queue, feedback_queue);
        connectionHandler.start();

		BPA[] node_uncertainties = null;
		ConfigurationOptions opts = null;
		float[] sensor_accuracies = null;

		logger.info("Waiting for data");
		boolean data_recieved = false;

		while(true){

			//-----------------------------------------INITIALIZE DATA-----------------------------------------

			while(data_recieved == false){
				try{
					semaphore.acquire();
					node_uncertainties = uncertainty_queue.peek();
					opts = config_queue.peek();
					sensor_accuracies = sensor_acc_queue.peek();
					semaphore.release();
				} catch (InterruptedException e){
					e.printStackTrace();
				}

				if(node_uncertainties != null && opts != null && sensor_accuracies != null){
					logger.info("recieved all needed data");
					data_recieved = true;
				}
				else try{
					Thread.sleep(100);
				} catch (InterruptedException e) {}
			}

			//---------------------------------------TEST EACH STRATEGY---------------------------------------

			int iterations = 10;
			for(int i = 0; i < iterations; i++){
				try{
					semaphore.acquire();
					node_uncertainties = uncertainty_queue.peek();
					opts = config_queue.peek();
					sensor_accuracies = sensor_acc_queue.peek();
					semaphore.release();
				} catch (InterruptedException e){
					e.printStackTrace();
				}

				for(int k = 0; k < strategies.length; k ++)
				{
					result_list[k] = trySimulation(k, opts, uncertainty_queue.peek(), sensor_acc_queue.peek());
					if(result_list[k]){
						totals[k] ++;
					}
				}
			}

			float[] cur_pref = new float[strategies.length];

			//---------------------------------------UPDATE PREFERENCE---------------------------------------

			for(int j = 0; j < strategies.length; j ++)
			{
				double new_mean;

				if(bayesianUpdateBeta(iterations, totals[j], strategy_betas[j]).getMean() >= 0.99d){
					new_mean = 0.99;
				} else{
					new_mean = bayesianUpdateBeta(iterations, totals[j], strategy_betas[j]).getMean();
				}
				cur_pref[j] = (float)new_mean;
				
				strategy_betas[j] = BetaDistribution.of(1 + new_mean, 1 - new_mean);
				totals[j] = 0;
			}
			try{
				strategy_queue.clear();
				strategy_queue.put(cur_pref);
				
			} catch(InterruptedException e) {
				e.printStackTrace();
			}
		}
	}
}
