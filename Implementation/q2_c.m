% This script runs Q2(c)

% Create the configuration object.
configuration = drivebot.SimulatorConfiguration();

% Enable the laser to support pure SLAM
configuration.enableLaser = true;

% If you set this parameter to false, the simulator generates measurements
% with no noise in them. You might find this useful for debugging.
% However, unless specified otherwise, any submitted results must have this
% value set to true.
configuration.perturbWithNoise = true;

% Set up the simulator
simulator = drivebot.DriveBotSimulator(configuration, 'q2_c');

% Create the localization system
optimizationPeriod = inf;
drivebotSLAMSystem = drivebot.DriveBotSLAMSystem(configuration);
drivebotSLAMSystem.setRecommendOptimizationPeriod(optimizationPeriod);

% This tells the SLAM system to do a very detailed check that the input
% appears to be correct but can make the code run slowly. Once you are
% confident your code is working safely, you can set this to false.
drivebotSLAMSystem.setValidateGraph(false);

% Run the main loop and correct results
results = minislam.mainLoop(simulator, drivebotSLAMSystem);

% Minimal output plots. For your answers, please provide titles and label
% the axes.

%Make folder for plot if not exists
mkdir("Figures/2c_ii")


% Adding labels to the simulator output
title('Simulator Output')
xlabel('x position')
ylabel('y position')
saveas(gcf,'Figures/2c_ii/2.6 q2_c sim_output.png')


% Plot optimisation times if optimizationPeriod is not inf
if (optimizationPeriod < inf)
    minislam.graphics.FigureManager.getFigure('Optimization times');
    clf
    plot(results{1}.optimizationTimes, '*')
    legend('Optimsation Time', 'Location', 'best')
    title('Optimization times')
    xlabel('Timestep')
    ylabel('Optimisation Time (sec)')
    hold on
    saveas(gcf,'Figures/2c_ii/q2_c opt_time.png')
end


% Plot the error curves
minislam.graphics.FigureManager.getFigure('Errors');
clf
plot(results{1}.vehicleStateTime, results{1}.vehicleStateHistory'-results{1}.vehicleTrueStateHistory')
legend('x error', 'y error', 'theta error', 'Location', 'best')
title('Vehicle State Errors')
xlabel('Timestep')
ylabel('Error')
hold on
saveas(gcf,'Figures/2c_ii/2.7 q2_c veh_error.png')



% Plot covariance
minislam.graphics.FigureManager.getFigure('Vehicle Covariances');
clf
plot(results{1}.vehicleCovarianceHistory')
legend('x covariance', 'y covariance', 'theta covariance', 'Location', 'best')
title('Vehicle Covariances')
xlabel('Timestep')
ylabel('Covariance')
hold on
saveas(gcf,'Figures/2c_ii/2.8 q2_c veh_cov.png')

%only plot chi2 if the optimizationPeriod is not inf
if (optimizationPeriod < inf)
    % Plot chi2 values
    minislam.graphics.FigureManager.getFigure('chi2 values');
    clf
    plot(results{1}.chi2Time, results{1}.chi2History)
    legend('Chi^2 values', 'Location', 'best')
    title('Chi2 Values')
    xlabel('Timestep')
    ylabel('Chi2')
    hold on
    saveas(gcf, 'Figures/2c_ii/q2_c chi2.png')
end


% This is how to extract the graph from the optimizer
graph = drivebotSLAMSystem.optimizer();

% This is how to extract cell arrays of the vertices and edges from the
% graph
allVertices = graph.vertices();
allEdges = graph.edges();

% Work out the number of vehicle poses and landmarks. 
numVehicleVertices = 0;
numLandmarks = 0;

landmarkObservationsPerVehicleVertex = 0;
observationsPerLandmarkVertex = 0;

% Q2c:
% Finish implementing the code to capture information about the graph
% structure.

% We are using "cellfun" to apply "isclass" function to allVertices and create a binary mask array
% It is similar to map() in python. If the element in the cell array is
% VehicleStateVertex, the corresponding element in the binary array will be
% 1. After that, we simply count "how many element is 1" in the array to
% optbtain the total number of of vehicle
vehicleVerticesMask = cellfun('isclass',allVertices,"drivebot.graph.VehicleStateVertex");
numVehicleVertices = sum(vehicleVerticesMask);

% Apply the same logic to obtain the number of landmarks initalized
landmarkVerticesMask = cellfun('isclass',allVertices,"drivebot.graph.LandmarkStateVertex");
numLandmarks = sum(landmarkVerticesMask);

fprintf("Total number of vehicle poses stored: %d\n",numVehicleVertices);
fprintf("Total number of landmarks: %d\n",numLandmarks);

%part 2 calculate the average number of observations
allVehicleVertices = allVertices(vehicleVerticesMask);
allLandmarks = allVertices(landmarkVerticesMask);
totalObservation = 0;
%calculate total number of LandmarkRangeBearingEdge for all timestep 
for i = 1:length(allVehicleVertices)
    v = allVehicleVertices{i};
    totalObservation = totalObservation + sum(cellfun('isclass',v.edges,"drivebot.graph.LandmarkRangeBearingEdge"));
end
landmarkObservationsPerVehicleVertex = totalObservation/numVehicleVertices;
fprintf("Average number of observations made by a robot at each timestep: %d\n",landmarkObservationsPerVehicleVertex );

%average number of observations received by each landmark:
observationsPerLandmarkVertex = mean(cellfun(@(b)b.numberOfEdges,allLandmarks));
fprintf("Average number of observations received by each landmark: %d\n",observationsPerLandmarkVertex );

