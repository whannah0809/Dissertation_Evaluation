This directory is the project file for carrying out the adaptive decision making performance testing for 
different routing implementations. To run, install ant as described in the Adasim folder README and follow:

1. Navigate to the Decision Making Experiments/adasim_DIGITAL_TWIN directory.
2. Run:

ant compile

ant jar-deps

java -jar adasim-1.0.0-deps.jar

3. Navigate to the Decision Making Experiments/adasim_CLIENT directory.
4. Run:

ant compile

ant jar-deps

java -jar adasim-1.0.0-deps.jar n

where n is the index of the strategy to be tested: 
0. Strategy_0 
1. Strategy_1
2. Strategy_2
3. Strategy_3
4. Digital Twin Structure

5. Results will be stored in Decision Making Experiments/adasim_CLIENT/data
--> If results are not updating, delete the data file and rerun.