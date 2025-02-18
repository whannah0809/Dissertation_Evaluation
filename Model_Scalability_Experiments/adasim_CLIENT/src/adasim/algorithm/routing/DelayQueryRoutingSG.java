package adasim.algorithm.routing;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

import org.apache.log4j.Logger;

import adasim.model.RoadSegment;
import adasim.model.Vehicle;
import adasim.uncertainty_handling.BPA;
import adasim.uncertainty_handling.Interval;

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
 * Routing strategy described as Strategy_0 in the paper when num_samples = 0. 
 * Takes a parameter for number of samples, when num_samples = 1, the 
 * strategy is Strategy_1
 * 
 * @author William Hannah
 * 
 **/

public class DelayQueryRoutingSG implements UncertainRoutingAlgorithm {

    private BPA[] bpas;
	private float[] sensor_accuracies;
	private List<int[]> possible_readings = new ArrayList<>();

	private int reading_unc = 1;

    public DelayQueryRoutingSG(int num_samples, BPA[] bpas, float[] sensor_accuracies)
    {
        this.bpas = bpas;
		this.sensor_accuracies = sensor_accuracies;
		this.reading_unc = num_samples;
    }	

	/**
	 * Finds a path from the source to the target considering delay uncertainties.
	 * 
	 * @param nodes The list of road segments representing the graph.
	 * @param source The source road segment.
	 * @param target The target road segment.
	 * 
	 * @return The path from the source to the target considering delay uncertainties.
	 * 
	 * @throws InterruptedException If the thread is interrupted during the computation.
	 * 
	 * @author William Hannah
	 */
	public List<RoadSegment> pathFind(List<RoadSegment> nodes, RoadSegment source, RoadSegment target)
	{
		for(int i = 0; i < sensor_accuracies.length - 1; i++){
			int reading = nodes.get(i).estimateDelay(sensor_accuracies[i]);
			int uncertainty = (int)sensor_accuracies[i];

			int numValues = reading_unc*2 + 1;
			int[] range = new int[numValues];
			
			double stepSize = (double) (uncertainty*2) / (numValues - 1.0d);
			if (Thread.currentThread().isInterrupted()) {
				return null;
			}


			for (int j = 0; j < numValues; j++) {
				range[j] = Math.max(0, (int) (reading - uncertainty + j * stepSize));
			}

			possible_readings.add(range);
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
	 * Generates combinations from a list of arrays.
	 * 
	 * @param listOfArrays The list of arrays to generate combinations from.
	 * 
	 * @return The list of combinations generated from the input list of arrays.
	 * 
	 * @throws InterruptedException If the thread is interrupted during the computation.
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
	 * Recursively generates combinations from a list of arrays.
	 * 
	 * @param listOfArrays The list of arrays to generate combinations from.
	 * @param index The current index being processed.
	 * @param currentCombination The current combination being generated.
	 * @param result The list to store the generated combinations.
	 * 
	 * @throws InterruptedException If the thread is interrupted during the computation.
	 * 
	 * @author William Hannah
	 */
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
}