package adasim.uncertainty_handling;

import java.util.*;

/*******************************************************************************
 *Created for Lancaster University Third Year Project.
 ********************************************************************************
 * William Hannah
 * Submitted: Mar 21, 2024
 */

/**
 * Class representing the simulation outcomes
 * @author William Hannah
 *
 */

public class SimulationResults {
    public List<Integer> path_taken;
    public boolean correct_path;
    public List<Integer> delay_observations;

    /**
     * Constructs a SimulationResults object with the specified parameters.
     * 
     * @param path_taken The path taken during the simulation.
     * @param correct_path Indicates whether the path taken is correct.
     * @param delay_observations The delay observations made during the simulation.
     * 
     * @return A new SimulationResults object.
     * 
     * @author William Hannah
     */
    public SimulationResults(List<Integer> path_taken, boolean correct_path, List<Integer> delay_observations)
    {
        this.path_taken = path_taken;
        this.correct_path = correct_path;
        this.delay_observations = delay_observations;
    }
}