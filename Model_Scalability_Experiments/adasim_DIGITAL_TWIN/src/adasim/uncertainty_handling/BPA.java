package adasim.uncertainty_handling;
import java.io.Serializable;

import java.util.*;

/*******************************************************************************
 *Created for Lancaster University Third Year Project.
 ********************************************************************************
 * William Hannah
 * Submitted: Mar 21, 2024
 */

/**
 * BPA class for representing delays for a given node on the map
 * Also keeps track of evidence used in DST regarding this node.
 * 
 * @author William Hannah
 * 
 **/
public class BPA implements Serializable {

    private List<TreeMap<Integer, Float>> evidence_mapping;
    private TreeMap<Integer, Float> delay_uncertainty = new TreeMap<>();
    private float[] bel;
    private int num_evidence;
    private int interval_size = 10;
    private float[][] evidence_list;

    /**
     * Constructs a BPA with node uncertainty and evidence list.
     * 
     * @param node_uncertainty Array of node uncertainty.
     * @param evidence_list    Array of evidence lists.
     * 
     * @author William Hannah
     */
    public BPA(float[] node_uncertainty, float[][] evidence_list){
        this.evidence_list = evidence_list;
        SetMapping(delay_uncertainty, node_uncertainty);
        num_evidence = evidence_list.length;
        bel = new float[num_evidence];
        evidence_mapping = new ArrayList<>();
        for (int i = 0; i < num_evidence; i++) {
            evidence_mapping.add(new TreeMap<>());
            SetMapping(evidence_mapping.get(i), evidence_list[i]);
            bel[i] = 1f/num_evidence;
        }
    }

    /**
     * Sets the mapping for a given distribution.
     * 
     * @param mapping The mapping to be set.
     * @param distribution The distribution to set mapping for.
     * 
     * @author William Hannah
     */
    public void SetMapping(TreeMap<Integer, Float> mapping, float[] distribution)
    {
        mapping.clear();
        for(int i = 0; i < interval_size; i ++)
        {
            mapping.put(i*interval_size + 1, distribution[i]);
            mapping.put(i*interval_size + 11, distribution[i]);
        }
    }

    /**
     * Updates the distributions based on the reading.
     * 
     * @param reading The reading used for updating.
     * 
     * @author William Hannah
     */
    public void updateDistributions(int reading){
        if(reading != 0){
            reading -= 1;
        }
        reading = Math.min(reading, 9);

        float total_bel = 0;
        float total_prob = 0;
        float prevbel = bel[1];
        for (int i = 0; i < num_evidence; i++) {
            float[] cur_dist = getDistribution(evidence_mapping.get(i));
            bel[i] *= cur_dist[reading];
            total_prob += cur_dist[reading];
            total_bel += bel[i];
        }

        for (int j = 0; j < num_evidence; j++) {
            bel[j] /= total_bel;
        }
    }

    /**
     * Retrieves the distributions.
     * 
     * @return Array of distributions.
     * 
     * @author William Hannah
     */
    public float[][] getDistributions() {
        int size = evidence_mapping.size();
        float[][] distributions = new float[size][10];
        for (int i = 0; i < size; i++) {
            TreeMap<Integer, Float> mapping = evidence_mapping.get(i);
            float[] distribution = getDistribution(mapping);
            distributions[i] = distribution;
        }
        return distributions;
    }

    /**
     * Retrieves a distribution from the mapping.
     * 
     * @param mapping The mapping to retrieve distribution from.
     * 
     * @return The retrieved distribution.
     * 
     * @author William Hannah
     */
    public static float[] getDistribution(TreeMap<Integer, Float> mapping) {
        float[] distribution = new float[10];
        for (int i = 0; i < 10; i++) {
            float value = mapping.get(i * 10 + 1);
            distribution[i] = value;
        }
        return distribution;
    }

    /**
     * Generates a delay based on uncertainty distribution.
     * 
     * @return The generated delay.
     * 
     * @author William Hannah
     */
    public int GenerateDelay() {
        float totalProbability = 0.0f;
        for (float probability : delay_uncertainty.values()) {
            totalProbability += probability;
        }

        TreeMap<Integer, Float> cumulativeProbabilities = new TreeMap<>();
        float cumulativeProbability = 0.0f;
        for (Map.Entry<Integer, Float> entry : delay_uncertainty.entrySet()) {
            cumulativeProbability += entry.getValue() / totalProbability; // Normalize probabilities
            cumulativeProbabilities.put(entry.getKey(), cumulativeProbability);
        }

        Random random = new Random();
        float randomNumber = random.nextFloat();

        int selectedKey = 0;
        for (Map.Entry<Integer, Float> entry : cumulativeProbabilities.entrySet()) {
            if (randomNumber <= entry.getValue()) {
                selectedKey = entry.getKey(); 
                break; 
            }
        }
        int delay = (int) selectedKey + random.nextInt(10);
        return delay;
    }

    /**
     * Retrieves the interval size.
     * 
     * @return The interval size.
     * 
     * @author William Hannah
     */
    public int getIntervalSize(){
        return interval_size;
    }

    /**
     * Retrieves the interval size.
     * 
     * @return The interval size.
     * 
     * @author William Hannah
     */
    public BPA generateSendableBPA(){
        float[] average_unc = new float[evidence_list[0].length];

        for(int i = 0; i < evidence_list[0].length; i++){
            float average = 0;
            for(int j = 0; j < num_evidence; j++){
                average += (bel[j] * evidence_list[j][i]);
            }
            average_unc[i] = average/3f; 
        }

        return new BPA(average_unc, evidence_list);
    }

}
