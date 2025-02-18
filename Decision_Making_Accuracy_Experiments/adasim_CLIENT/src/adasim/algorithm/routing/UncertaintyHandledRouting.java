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

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

import org.apache.log4j.Logger;

import adasim.model.RoadSegment;
import adasim.model.Vehicle;
import adasim.uncertainty_handling.BPA;

import java.io.*;
import java.net.*;

/*******************************************************************************
 *Created for Lancaster University Third Year Project.
 ********************************************************************************
 * William Hannah
 * Submitted: Mar 21, 2024
 */

/**
 * Routing strategy that integrates a digital twin
 * @author William Hannah
 *
 */
public class UncertaintyHandledRouting extends AbstractRoutingAlgorithm {
	
	private final static Logger logger = Logger.getLogger(LookaheadShortestPathRoutingAlgorithm.class);
	
	private final int lookahead = 0;
	private final int recompute;
	private List<RoadSegment> path;
	private int steps;
	private int strategy;
	private boolean finished = false;
	private BPA[] bpas;
	private float[] sensor_accuracies;
	
	/**
     * Initializes UncertaintyHandledRouting with the specified parameters.
     * 
     * @param bpas Array of belief probability assignments.
	 * @param strategy id of the strategy to be implemented
     * @param sensor_accuracies Array of sensor accuracies.
	 * 
	 * @author William Hannah
     */
	public UncertaintyHandledRouting(BPA[] bpas, float[] sensor_accuracies, int strategy){
		this.sensor_accuracies = sensor_accuracies;
		this.bpas = bpas;
		this.recompute = 1;
		this.steps = 0;
		this.strategy = strategy;
		logger.info( "UncertaintyHandledRouting(" + lookahead + "," + recompute +")" );
	}

	/**
	 * Gets the uncertain routing algorithm based on the specified strategy.
	 * 
	 * @return The uncertain routing algorithm based on the specified strategy.
	 * 
	 * @author William Hannah
	 */
	UncertainRoutingAlgorithm getStrategy(){
		UncertainRoutingAlgorithm r_algorithm = new DelayEstimateRouting(bpas, 1, 0.9f);
		switch(strategy){
			case 0:
				r_algorithm = new DelayEstimateRouting(bpas, 0, 0.7f);
				break;
			case 1:
				r_algorithm = new DelayEstimateRouting(bpas, 1, 0.9f);
				break;
			case 2:
				r_algorithm = new DelayQueryRoutingSG(0, bpas, sensor_accuracies);
				break;
			case 3:
				r_algorithm = new DelayQueryRoutingSG(1, bpas, sensor_accuracies);
				break;
		}

		return r_algorithm;
	}
	
	/**
     * Retrieves the estimated path.
     * 
     * @return The the estimated path.
     * 
     * @author William Hannah
     */
	public List<RoadSegment> getPath(RoadSegment source, RoadSegment target ) {
		UncertainRoutingAlgorithm u_routing = getStrategy();
		return u_routing.pathFind(graph.getRoadSegments(), source, target );
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
		UncertainRoutingAlgorithm u_routing = getStrategy();
		List<RoadSegment> p = u_routing.pathFind(graph.getRoadSegments(), start, target );
		if ( p == null ) {
			finished = true;
		}
		return p;
	}

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
