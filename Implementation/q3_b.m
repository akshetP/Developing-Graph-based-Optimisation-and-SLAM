% Q3B

% In the report, we have discussed 2 graph puring approach that we tested.
% We implemented both of them. Feel free to update the parameter, "METHODS"
% below on line 12 to verify our result.


% The OPTION for simulation
% 0 = Full SLAM without any pruning
% 1 = Remove all the Observation edges after reaching the upper limit of
% the edge number
% 2 = Lower the resolution (For every 2 LandmarkRangeBearingEdge, remove 1
% of it)
METHODS = 2;

% Create the configuration object.
configuration = drivebot.SimulatorConfiguration();

% Enable the laser to support pure SLAM
configuration.enableLaser = true;

% If you set this parameter to false, the simulator generates measurements
% with no noise in them. You might find this useful for debugging.
% However, unless specified otherwise, any submitted results must have this
% value set to true.
configuration.perturbWithNoise = true;

% Magic tuning for the no-prediction case
configuration.laserDetectionRange = 30;

% Set up the simulator
% Keep q3_a here to use the simulation data from q3_a
simulator = drivebot.DriveBotSimulator(configuration, 'q3_a'); 

% Create the localization system, we inherit the origianl drivebot.DriveBotSLAMSystem class to 
% seperate the extra logic we added for this 3B
% The second parameter here is the number limit of edges. 
% We only use if the METHODS is 1. We picked 460 because that is the
% average number of observations received by each landmark which we found
% from Question 2C
drivebotSLAMSystem = drivebot.DriveBotSLAMSystem3B(configuration,460,METHODS);

% This tells the SLAM system to do a very detailed check that the input
% appears to be correct but can make the code run slowly. Once you are
% confident your code is working safely, you can set this to false.
drivebotSLAMSystem.setValidateGraph(false);

% Optimize every 500 timesteps to get a picture of the situation as it evolves
drivebotSLAMSystem.setRecommendOptimizationPeriod(500);


% Run the main loop and correct results
results = minislam.mainLoop(simulator, drivebotSLAMSystem);

% Minimal output plots. For your answers, please provide titles and label
% the axes.

mkdir("Figures/3b")



% Adding labels to the simulator output
title('Simulator Output')
xlabel('x position')
ylabel('y position')
saveas(gcf,'Figures/3b/q3_b_sim_out.png')


% Plot optimisation times
minislam.graphics.FigureManager.getFigure('Optimization times');
clf
plot(results{1}.optimizationTimes, '*')
legend('Optimsation Time', 'Location', 'best')
title('Optimization times')
xlabel('Timestep')
ylabel('Optimisation Time (sec)')
hold on
saveas(gcf,'Figures/3b/q3_b_opt_times.png')


% Plot the error curves
minislam.graphics.FigureManager.getFigure('Errors');
clf
plot(results{1}.vehicleStateHistory'-results{1}.vehicleStateHistory')

% Plot covariance
minislam.graphics.FigureManager.getFigure('Vehicle Covariances');
clf
plot(results{1}.vehicleCovarianceHistory')
legend('x covariance', 'y covariance', 'theta covariance', 'Location', 'best')
title('Vehicle Covariances')
xlabel('Timestep')
ylabel('Covariance')
hold on
saveas(gcf,'Figures/3b/q3_b_covariances.png')


% Plot errors
minislam.graphics.FigureManager.getFigure('Errors');
clf
plot(results{1}.vehicleStateHistory'-results{1}.vehicleTrueStateHistory')
legend('x error', 'y error', 'theta error', 'Location', 'best')
title('Vehicle State Errors')
xlabel('Timestep')
ylabel('Error')
hold on
saveas(gcf,'Figures/3b/q3_b_errors.png')



% Reuse our solution from Question 2C to compute the following infomration for analysis:
% - Number of vehicle poses stored
% - The number of landmarks initalized
% - The average number of observations made by a robot at each timestep
% - The average number of observations received by each landmark

% Extract cell arrays of the vertices and edges from the graph
graph = drivebotSLAMSystem.optimizer();
allVertices = graph.vertices();
allEdges = graph.edges();


% Total number of vertices and landmarks
% Similar to map() in python, we apply the "isclass" function to the
% allVertices array to create a binary mask, 1 = the object in the cell is
% an instance of "drivebot.graph.VehicleStateVertex"
vehicleVerticesMask = cellfun('isclass',allVertices,"drivebot.graph.VehicleStateVertex");
% Count how many cells in the array are "drivebot.graph.VehicleStateVertex"
numVehicleVertices = sum(vehicleVerticesMask);

% Compute how many landmark vertex in the graph
landmarkVerticesMask = cellfun('isclass',allVertices,"drivebot.graph.LandmarkStateVertex");
numLandmarks = sum(landmarkVerticesMask);

fprintf("Total number of vehicle poses stored: %d\n",numVehicleVertices);
fprintf("Total number of landmarks: %d\n",numLandmarks);

% Calculate the average number of observations

% Filter the vertices array and get all Vehicle Vertices
allVehicleVertices = allVertices(vehicleVerticesMask);

% Filter the vertices array and get all Landmark Vertices
allLandmarks = allVertices(landmarkVerticesMask);

% Calculate total number of VehicleKinematicsEdge for all timestep 
totalObservation = 1;
for i = 1:length(allVehicleVertices)
    v = allVehicleVertices{i};
    totalObservation = totalObservation + sum(cellfun('isclass',v.edges,"drivebot.graph.LandmarkRangeBearingEdge"));
end

% Compute the average number of observations made by a robot at each
% timesteps
landmarkObservationsPerVehicleVertex = totalObservation/numVehicleVertices;
fprintf("Average number of observations made by a robot at each timestep: %d\n",landmarkObservationsPerVehicleVertex );

% Compute the average number of observations received by each landmark:
observationsPerLandmarkVertex = mean(cellfun(@(b)b.numberOfEdges,allLandmarks));
fprintf("Average number of observations received by each landmark: %d\n",observationsPerLandmarkVertex );



