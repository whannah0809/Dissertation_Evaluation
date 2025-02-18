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
import java.util.*;
import java.net.*;

/*******************************************************************************
 *Created for Lancaster University Third Year Project.
 ********************************************************************************
 * William Hannah
 * Submitted: Mar 21, 2024
 */

/**
 * Routing strategy described as Strategy_0 in the paper. Takes a parameter for
 * number of samples, when num_samples = 1, the strategy is Strategy_1
 * 
 * @author William Hannah
 * 
 **/

public class DelayEstimateRouting implements UncertainRoutingAlgorithm {

    private BPA[] bpas;
	int num_samples;
	float confidence_interval;
	private List<int[]> possible_readings = new ArrayList<>();

	/**
	 * @param bpas the uncertainty of delays at nodes
	 * @param num_samples the number of samples to take in the confidence interval of the estimated delay
	 * @param confidence_interval the percentage confidence for the interval
	 * 
	 * @author William Hannah
	 */
    public DelayEstimateRouting(BPA[] bpas, int num_samples, float confidence_interval)
    {
        this.bpas = bpas;
		this.num_samples = num_samples;
		this.confidence_interval = confidence_interval;
    }

	/**
	 * @param nodes the nodes of the map
	 * @param source the starting node of the agent
	 * @param target the target node
	 * 
	 * @return the optimal path estimated
	 * 
	 * @author William Hannah
	 */
	public List<RoadSegment> pathFind(List<RoadSegment> nodes, RoadSegment source, RoadSegment target) {
		for(int i = 0; i < nodes.size(); i++){
			possible_readings.add(estimateDelay(nodes.get(i), num_samples, confidence_interval));
			if (Thread.currentThread().isInterrupted()) {
				return null;
			}
		}

		try{
			List<int[]> possible_delays = generateCombinations(possible_readings);


			List<List<RoadSegment>> simulatedPaths = new ArrayList<>();

			for (int[] combination : possible_delays) {
				simulatedPaths.add(dijkstra(nodes, source, target, combination));
				if (Thread.currentThread().isInterrupted()) {
					return null;
				}
			}

			Map<List<RoadSegment>, Integer> frequencyMap = new HashMap<>();

			for (List<RoadSegment> path : simulatedPaths) {
				frequencyMap.put(path, frequencyMap.getOrDefault(path, 0) + 1);
			}

			List<RoadSegment> mostFrequentPath = null;
			int maxFrequency = 0;
			for (Map.Entry<List<RoadSegment>, Integer> entry : frequencyMap.entrySet()) {
				if (entry.getValue() > maxFrequency) {
					maxFrequency = entry.getValue();
					mostFrequentPath = entry.getKey();
				}
				if (Thread.currentThread().isInterrupted()) {
					return null;
				}
			}

			return(mostFrequentPath);
		} catch(InterruptedException e){
			return null;
		}
	}

	/**
     * Generates combinations of delays.
     * 
     * @param listOfArrays List of arrays representing possible delays.
     * @return List of combinations of delays.
     * @throws InterruptedException If the thread is interrupted while generating combinations.
	 * 
	 * @author William Hannah
     */
	public static List<int[]> generateCombinations(List<int[]> listOfArrays) throws InterruptedException {
        List<int[]> result = new ArrayList<>();
        try {
            recursiveCombinationAdder(listOfArrays, 0, new int[listOfArrays.size()], result);
        } catch (InterruptedException e) {
            return null;
        }
        return result;
    }

	/**
     * Recursively generates combinations of arrays.
     * 
     * @param listOfArrays       List of arrays to combine.
     * @param index              Current index during recursion.
     * @param currentCombination Current combination being constructed.
     * @param result             Result list to store generated combinations.
     * @throws InterruptedException If the thread is interrupted during recursion.
	 * 
	 * @author William Hannah
	 **/
    private static void recursiveCombinationAdder(List<int[]> listOfArrays, int index, int[] currentCombination, List<int[]> result) throws InterruptedException {
        if (Thread.currentThread().isInterrupted()) {
            throw new InterruptedException();
        }

        if (index == listOfArrays.size()) {
            result.add(currentCombination.clone());
            return;
        }

        int[] currentArray = listOfArrays.get(index);
        for (int num : currentArray) {
            currentCombination[index] = num;
			try {
				recursiveCombinationAdder(listOfArrays, index + 1, currentCombination, result);
			} catch (InterruptedException e) {
				return;
			}
        }
    }

	/**
 	 * @author Jochen Wuttke - wuttkej@gmail.com
	 */
    public List<RoadSegment> dijkstra(List<RoadSegment> nodes, RoadSegment source, RoadSegment target, int[] delays) {
		int size = nodes.size();
		int[] dist = new int[size];
		int[] previous = new int[size];
		Set<Integer> q = new HashSet<Integer>();
		
		init( dist, previous, getIndex(nodes, source) , q );
		while( !q.isEmpty() ) {
			int current = getIndexOfMin(q, dist);
			if ( dist[current] == Integer.MAX_VALUE ) break;
			q.remove( current );
			
			for ( RoadSegment node : nodes.get(current).getNeighbors() ) {
				int depth = getCurrentDepth(previous, nodes, source, nodes.get(current) );
				
				//if we ever make vehicle extensible, then we have to query the class of the configure vehicle
				int t = dist[current] + delays[current];
				int thisIndex = getIndex( nodes, node );
				if ( t < dist[ thisIndex ] ) {
					dist[thisIndex] = t;
					previous[thisIndex] = current;
				}
			}
		}
		return reconstructPath( previous, nodes, source, target );
	}

	/**
	 * @param previous
	 * @param nodes
	 * @param source
	 * @param current
	 * @return the current depth of the search path
	 * 
	 * @author Jochen Wuttke - wuttkej@gmail.com
	 */
	private int getCurrentDepth(int[] previous, List<RoadSegment> nodes,
			RoadSegment source, RoadSegment current) {
		List<RoadSegment> path = reconstructPath(previous, nodes, source, current );
		if ( path == null ) return 1;
		else return path.size() + 1;
	}

	/**
	 * @param previous map to previous nodes on a path 
	 * @param nodes list of all nodes
	 * @param source ID of source node
	 * @param target ID of target node
	 * @return the path constructed from the intermediate data structures passed in
	 * 
	 * @author Jochen Wuttke - wuttkej@gmail.com
	 */
	private List<RoadSegment> reconstructPath(int[] previous, List<RoadSegment> nodes, RoadSegment source, RoadSegment target) {
		int ti = getIndex(nodes, target);
		if ( previous[ ti ] == -1 ) return null; //no path
		LinkedList<RoadSegment> path = new LinkedList<RoadSegment>();
		int current = ti;
		do {
			path.push( nodes.get(current) );
			current = previous[current];
		} while ( current != getIndex(nodes, source) && previous[current] != -1 );
		return path;
	}

	/**
	 * @param nodes
	 * @param node
	 * @return the index of the node in the list, -1 if the node cannot be found
	 * 
	 * @author Jochen Wuttke - wuttkej@gmail.com
	 */
	private int getIndex(List<RoadSegment> nodes, RoadSegment node) {
		for ( int i =0 ; i < nodes.size() ; i++ ) {
			if ( nodes.get(i).equals( node ) ) return i;
		}
			
		return -1;
	}

	/**
	 * Computes the array index of the smallest element
	 * @param q
	 * @return the index of the smalles element
	 * 
 	 * @author Jochen Wuttke - wuttkej@gmail.com
	 */
	private int getIndexOfMin(Set<Integer> q, int[] dist) {
		int min = q.iterator().next();
		for ( int i : q ) {
			if ( dist[min] > dist[i] ) min = i;
		}
		return min;
	}

	/**
	 * @param dist
	 * @param previous
	 * @param source
	 * 
 	 * @author Jochen Wuttke - wuttkej@gmail.com
	 */
	private void init(int[] dist, int[] previous, int source, Set<Integer> q) {
		for ( int i = 0; i < dist.length; i++ ) {
			if ( i == source ) {
				dist[i] = 0;
			} else {
				dist[i] = Integer.MAX_VALUE;
			}
			previous[i] = -1;
			q.add(i);
		}
	}

	/**
     * Estimates delay for a road segment.
     * 
     * @param node The road segment.
     * @param num_samples Number of samples.
     * @param confidence_interval Confidence interval.
     * @return Range of delays.
	 * 
	 * @author William Hannah
     */
    public int[] estimateDelay(RoadSegment node, int num_samples, float confidence_interval)
    {
        int id = node.getID();

        float[] combined_belief = dempsterCombination(bpas[id].getDistributions());

		float sum = 0f;
		int current_range = 0;
		float[] cumulativeDistribution = new float[combined_belief.length * bpas[id].getIntervalSize()];

        for (int i = 0; i < combined_belief.length * bpas[id].getIntervalSize(); i++) {
			if (Thread.currentThread().isInterrupted()) {
				return null;
			}
			if(i - bpas[id].getIntervalSize() > current_range * bpas[id].getIntervalSize()){
				current_range += 1;
			}
			sum += combined_belief[current_range]/bpas[id].getIntervalSize();
			cumulativeDistribution[i] = sum;
		}

		float lowerBound = (1 - confidence_interval)/2f;
		float upperBound = confidence_interval + lowerBound;

		int lowerIndex = 0;
		int upperIndex = 0;

		for (int i = 0; i < cumulativeDistribution.length; i++) {
			if (cumulativeDistribution[i] >= lowerBound) {
				lowerIndex = i;
				break;
			}
		}

		if (Thread.currentThread().isInterrupted()) {
			return null;
		}

		for (int i = 0; i < cumulativeDistribution.length; i++) {
			if (cumulativeDistribution[i] >= upperBound) {
				upperIndex = i;
				break;
			}
		}

		int est_value = (int)(upperIndex + lowerIndex)/2;
		int uncertainty = upperIndex - est_value;
		
		int numValues = num_samples*2 + 1;
		int[] range = new int[numValues];
		
		double stepSize = (double) (upperIndex - lowerIndex) / (numValues - 1.0d);
		
		for (int i = 0; i < numValues; i++) {
			range[i] = (int) (lowerIndex + i * stepSize);

			if (Thread.currentThread().isInterrupted()) {
				return null;
			}
		}

		for(int y = 0; y < numValues; y++){
			if (Thread.currentThread().isInterrupted()) {
				return null;
			}
		}
		return range;
	}

	/**
     * Combines belief functions using Dempster's rule.
     * 
     * @param bpas Array of belief functions.
     * @return Combined belief function.
	 * 
	 * @author William Hannah
     */
    public static float[] dempsterCombination(float[][] bpas) {
        int n = bpas[0].length;
        for (float[] bpa : bpas) {
            if (bpa.length != n) {
                throw new IllegalArgumentException("Arrays must have the same length");
            }
        }

        float[] combinedBelief = new float[n];
        boolean[] conflictIndices = new boolean[n];

        float denominator = 1.0f;
        for (int i = 0; i < n; i++) {
            boolean hasConflict = false;
            for (float[] bpa : bpas) {
                if (bpa[i] == 0) {
                    conflictIndices[i] = true;
                    hasConflict = true;
                    break;
                }
            }
            if (!hasConflict) {
                float product = 1.0f;
                for (float[] bpa : bpas) {
                    product *= (1 - bpa[i]);
                }
                denominator -= product;
            }
        }

        for (int i = 0; i < n; i++) {
            if (!conflictIndices[i]) {
                float numerator = 1.0f;
                for (float[] bpa : bpas) {
                    numerator *= bpa[i];
                }
                combinedBelief[i] = numerator / denominator;
            }
        }

        float sum = 0;
        for (float belief : combinedBelief) {
            sum += belief;
        }
        for (int i = 0; i < n; i++) {
            if (!conflictIndices[i]) {
                combinedBelief[i] /= sum;
            }
        }

        return combinedBelief;
    }
}