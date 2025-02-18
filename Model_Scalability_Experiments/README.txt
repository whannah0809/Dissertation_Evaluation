This directory is the project file for carrying out the scalability performance testing for 
different routing implementations. To run, install ant as described in the Adasim folder README and follow:

For testing the digital twin structure:
1. Navigate to the Model_Scalability_Experiments/adasim_DIGITAL_TWIN directory.
2. Run:

ant compile

ant jar-deps

java -jar adasim-1.0.0-deps.jar n

where n is the number of candidate models.

3. Navigate to the Model_Scalability_Experiments/adasim_CLIENT directory.
4. Run:

ant compile

ant jar-deps

java -jar adasim-1.0.0-deps.jar

5. Results will be stored in Model_Scalability_Experiments/adasim_CLIENT/data
--> If results are not updating, delete the data file and rerun.


For testing the model averaging structure:
1. Navigate to the Model_Scalability_Experiments/adasim_MODEL_AVERAGING directory.
2. Run:

ant compile

ant jar-deps

java -jar adasim-1.0.0-deps.jar n

where n is the number of candidate models.

3. Results will be stored in Model_Scalability_Experiments/adasim_MODEL_AVERAGING/data
--> If results are not updating, delete the data file and rerun.
