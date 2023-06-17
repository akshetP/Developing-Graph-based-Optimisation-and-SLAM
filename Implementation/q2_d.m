% This script runs Q2(d)


% Create the configuration object.
configuration = drivebot.SimulatorConfiguration();

% Enable the laser to support pure SLAM
configuration.enableLaser = true;

% For this part of the coursework, this should be set to false.
configuration.perturbWithNoise = false;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Run the simulation before loop closure happens

% Set this value to truncate the run at a specified timestep rather than
% run through the whole simulation to its end.
% loop closure happens at timestep 1208 here we set 1205, time step before
% loop closure
configuration.maximumStepNumber = 1205;

% Set up the simulator
simulator = drivebot.DriveBotSimulator(configuration, 'q2_d');

% Create the localization system before loop closure
drivebotSLAMSystem = drivebot.DriveBotSLAMSystem(configuration);
drivebotSLAMSystem.setRecommendOptimizationPeriod(inf);


% This tells the SLAM system to do a very detailed check that the input
% appears to be correct but can make the code run slowly. Once you are
% confident your code is working safely, you can set this to false.
drivebotSLAMSystem.setValidateGraph(false);

% Run the main loop and correct results
results_before = minislam.mainLoop(simulator, drivebotSLAMSystem);



%Make folder for plot if not exists
mkdir("Figures/2d")

% Adding labels to the simulator output
title('Simulator Output')
xlabel('x position')
ylabel('y position')
saveas(gcf,'Figures/2d/2d_simplot_before.png')


% Plot covariance.
minislam.graphics.FigureManager.getFigure('Vehicle Covariances');
clf
plot(results_before{1}.vehicleCovarianceHistory')
legend('x covariance', 'y covariance', 'theta covariance', 'Location', 'best')
title('Vehicle Covariances')
xlabel('Timestep')
ylabel('Covariance')
hold on
saveas(gcf,'Figures/2d/2d_covariances_before.png')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Re run the simulation after loop closure happens

% Set this value to truncate the run at a specified timestep rather than
% run through the whole simulation to its end.
%loop closure happens at timestep 1208
configuration.maximumStepNumber = 1208;

% Set up the simulator
simulator = drivebot.DriveBotSimulator(configuration, 'q2_d');

% Create the localization system before loop closure
drivebotSLAMSystem = drivebot.DriveBotSLAMSystem(configuration);
drivebotSLAMSystem.setRecommendOptimizationPeriod(inf);


% This tells the SLAM system to do a very detailed check that the input
% appears to be correct but can make the code run slowly. Once you are
% confident your code is working safely, you can set this to false.
drivebotSLAMSystem.setValidateGraph(false);

% Run the main loop and correct results
results_after = minislam.mainLoop(simulator, drivebotSLAMSystem);


% Adding labels to the simulator output
title('Simulator Output')
xlabel('x position')
ylabel('y position')
saveas(gcf,'Figures/2d/2d_simplot_after.png')


% Plot covariance.
minislam.graphics.FigureManager.getFigure('Vehicle Covariances');
clf
plot(results_after{1}.vehicleCovarianceHistory')
legend('x covariance', 'y covariance', 'theta covariance', 'Location', 'best')
title('Vehicle Covariances')
xlabel('Timestep')
ylabel('Covariance')
hold on
saveas(gcf,'Figures/2d/2d_covariances_after.png')

