package adasim.algorithm.delay;

import adasim.uncertainty_handling.BPA;

/*******************************************************************************
 *Created for Lancaster University Third Year Project.
 ********************************************************************************
 * William Hannah
 * Submitted: Mar 21, 2024
 */

/**
 * Delay function that accepts a BPA and sets the delay at the node equal to
 * a random number selected with the distribution described by the BPA
 * 
 * @author William Hannah
 * 
 **/
public class EpistemicallyUncertainDelayFunction implements TrafficDelayFunction {

	private BPA bpa;
	private int delay;

	public EpistemicallyUncertainDelayFunction(BPA bpa, int id){
		this.bpa = bpa;
		if(id == 0){
			this.delay = 0;
		}else{
			this.delay = bpa.GenerateDelay();
		}
	}

	public int getDelay(int weight, int capacity, int number) {
		return delay;
	}
}
