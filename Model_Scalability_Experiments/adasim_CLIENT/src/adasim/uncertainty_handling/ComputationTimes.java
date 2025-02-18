package adasim.uncertainty_handling;

import java.util.*;

/*******************************************************************************
 *Created for Lancaster University Third Year Project.
 ********************************************************************************
 * William Hannah
 * Submitted: Mar 21, 2024
 */

/**
 * Tracks computation times.
 * 
 * @author William Hannah
 */
public class ComputationTimes {
    public List<Float> computation_times;

    /**
     * Constructs a ComputationTimes object with an empty list of computation times.
     * 
     * @author William Hannah
     */
    public ComputationTimes(){
        this.computation_times = new ArrayList<>();
    }

    /**
     * Adds a computation time to the list.
     * 
     * @param time The computation time to add.
     * 
     * @author William Hannah
     */
    public void AddTime(float time){
        this.computation_times.add(time);
    }

    /**
     * Retrieves the list of computation times.
     * 
     * @return The list of computation times.
     * 
     * @author William Hannah
     */
    public List<Float> GetComputationTimes(){
        return this.computation_times;
    }
}