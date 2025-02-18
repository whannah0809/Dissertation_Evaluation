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
 *    Jochen Wuttke (wuttkej@gmail.com) - initial API and implementation
 ********************************************************************************
 *
 * Created: Dec 3, 2011
 */
package adasim.algorithm.routing;

import java.time.*;
import java.util.*;
import java.util.concurrent.*;

import org.apache.log4j.Logger;

import adasim.model.RoadSegment;
import adasim.model.Vehicle;
import adasim.uncertainty_handling.*;


import java.io.*;
import java.net.*;


/*******************************************************************************
 *Created for Lancaster University Third Year Project.
 ********************************************************************************
 * William Hannah
 * Submitted: Mar 21, 2024
 */

/**
 * Routing strategy that implements model averaging
 * @author William Hannah
 *
 */
public class UncertaintyHandledRouting extends AbstractRoutingAlgorithm {
	
	private final static Logger logger = Logger.getLogger(LookaheadShortestPathRoutingAlgorithm.class);
	private Semaphore simulationSemaphore = new Semaphore(1);
	
	private final int lookahead = 0;
	private final int recompute;
	private List<RoadSegment> path;
	private int steps;
	private int num_strat;
	private boolean finished = false;
	private BPA[] bpas;
	private float[] sensor_accuracies;
	private ComputationTimes times;
	
	/**
     * Initializes UncertaintyHandledRouting with the specified parameters.
     * 
     * @param bpas Array of belief probability assignments.
     * @param sensor_accuracies Array of sensor accuracies.
     * @param num_strat Number of strategies.
     * @param times ComputationTimes object for time tracking.
	 * 
	 * @author William Hannah
     */
	public UncertaintyHandledRouting(BPA[] bpas, float[] sensor_accuracies, int num_strat, ComputationTimes times){
		this.times = times;
		this.sensor_accuracies = sensor_accuracies;
		this.bpas = bpas;
		this.recompute = 1;
		this.steps = 0;
		this.num_strat = num_strat;
		logger.info( "UncertaintyHandledRouting(" + lookahead + "," + recompute +")" );
	}

	/**
     * Selects the most popular path from the given list of candidate paths.
     * 
     * @param list List of candidate paths.
     * @return The most popular path.
     * 
     * @author William Hannah
     */
    public static List<RoadSegment> pathSelection(List<List<RoadSegment>> list) {
        Map<List<RoadSegment>, Integer> counts = new HashMap<>();

        for (List<RoadSegment> candidatePath : list) {
            counts.put(candidatePath, counts.getOrDefault(candidatePath, 0) + 1);
        }

        List<RoadSegment> mostPopular = null;
        int maxCount = 0;
        for (Map.Entry<List<RoadSegment>, Integer> entry : counts.entrySet()) {
            if (entry.getValue() > maxCount) {
                mostPopular = entry.getKey();
                maxCount = entry.getValue();
            }
        }

        return mostPopular;
    }
	
	/**
     * Retrieves the estimated path.
     * 
     * @return The the estimated path.
     * 
     * @author William Hannah
     */
	public List<RoadSegment> getPath(RoadSegment source, RoadSegment target ) {
		long startTime = System.currentTimeMillis();
		List<List<RoadSegment>> candidateRoutes = new ArrayList<>();
		ExecutorService mainExecutor = Executors.newFixedThreadPool(num_strat);

        for (int i = 0; i < num_strat; i++) {
            final Duration timeout = Duration.ofMillis(80);

            try {
                simulationSemaphore.acquire();
            } catch (InterruptedException e) {
                return null;
            }

            mainExecutor.submit(() -> {
				ExecutorService executor = Executors.newSingleThreadExecutor();
				try {
					executor.submit(() -> {
						try {
							UncertainRoutingAlgorithm r_algorithm = new DelayEstimateRouting(bpas, 1, 0.9f);
							candidateRoutes.add(r_algorithm.pathFind(graph.getRoadSegments(), source, target));
						} finally {
							simulationSemaphore.release();
						}
					}).get(timeout.toMillis(), TimeUnit.MILLISECONDS);
				} catch (TimeoutException e) {
					
				} catch (InterruptedException | ExecutionException e) {
					e.printStackTrace();
				} finally {
					executor.shutdownNow();
				}
			});
        }

        mainExecutor.shutdown();

        try {
            mainExecutor.awaitTermination(1000, TimeUnit.MILLISECONDS);
        } catch (InterruptedException e) {
            long endTime = System.currentTimeMillis();
			long elapsedTime = 0;
			times.AddTime(elapsedTime);
			return null;
        }

		long endTime = System.currentTimeMillis();
		long elapsedTime = endTime - startTime;
		times.AddTime(elapsedTime);
        return pathSelection(candidateRoutes);
    }

	/* (non-Javadoc)
	 * @see adasim.algorithm.CarStrategy#getNextNode()
	 */
	@Override
	public RoadSegment getNextNode() {
		if ( finished ) return null;
		if ( path == null ) {
			path = getPath(source);
			if (Thread.currentThread().isInterrupted()) {
				return null;
			}
			logger.info( pathLogMessage() );
		}
		assert path != null || finished;
		if ( path == null || path.size() == 0 ) {
			finished = true;
			return null;
		}
		if ( ++steps == recompute ) {
			RoadSegment next = path.remove(0);
			path = getPath(next);
			if (Thread.currentThread().isInterrupted()) {
				return null;
			}
			logger.info( "UPDATE: " + pathLogMessage() );
			steps = 0;
			return next;
		} else {
			return path.remove(0);
		}
	}

	/**
	 * Computes a path to the configured target node starting from
	 * the passed <code>start</code> node.
	 * @param start
	 * 
 	 * @author Jochen Wuttke - wuttkej@gmail.com
	 */
	private List<RoadSegment> getPath(RoadSegment start) {
		List<RoadSegment> p = getPath(start, target);
		if ( p == null ) {
			finished = true;
		}
		return p;
	}

	/** 
	 * @author Jochen Wuttke - wuttkej@gmail.com
	 */
	private String pathLogMessage() {
		StringBuffer buf = new StringBuffer( "PATH: Vehicle: " );
		buf.append( vehicle.getID() );
		buf.append( " From: " );
		buf.append( source.getID() );
		buf.append( " To: " );
		buf.append( target.getID() );
		buf.append( " Path: " );
		buf.append( path == null ? "[]" : path );
		return buf.toString();
	}
}