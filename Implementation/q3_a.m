% This script runs Q3(a)

% Create the configuration object.
configuration = drivebot.SimulatorConfiguration();

% Enable the laser to support pure SLAM
configuration.enableGPS = false;
configuration.enableLaser = true;

% If you set this parameter to false, the simulator generates measurements
% with no noise in them. You might find this useful for debugging.
% However, unless specified otherwise, any submitted results must have this
% value set to true.
configuration.perturbWithNoise = true;

% Magic tuning for the no-prediction case
configuration.laserDetectionRange = 30;

% Set up the simulator
simulator = drivebot.DriveBotSimulator(configuration, 'q3_a');

% Create the localization system
drivebotSLAMSystem = drivebot.DriveBotSLAMSystem(configuration);

% This tells the SLAM system to do a very detailed check that the input
% appears to be correct but can make the code run slowly. Once you are
% confident your code is working safely, you can set this to false.
drivebotSLAMSystem.setValidateGraph(false);

% Optimize every 500 timesteps to get a picture of the situation as it evolves
drivebotSLAMSystem.setRecommendOptimizationPeriod(500);

% Set whether the SLAM system should remove prediction edges. If the first
% value is true, the SLAM system should remove the edges. If the second is
% true, the first prediction edge will be retained.
drivebotSLAMSystem.setRemovePredictionEdges(true, true);

% Run the main loop and correct results
results = minislam.mainLoop(simulator, drivebotSLAMSystem);

% Minimal output plots. For your answers, please provide titles and label
% the axes.
mkdir("Figures/3a")

% Adding labels to the simulator output
title('Simulator Output')
xlabel('x position')
ylabel('y position')
saveas(gcf,'Figures/3a/q3_a_sim_out.png')

% Plot optimisation times
minislam.graphics.FigureManager.getFigure('Optimization times');
clf
plot(results{1}.optimizationTimes, '*')
legend('Optimsation Time', 'Location', 'best')
title('Optimization times')
xlabel('Timestep')
ylabel('Optimisation Time (sec)')
hold on
saveas(gcf,'Figures/3a/q3_a_opt_times.png')


% Plot covariance
minislam.graphics.FigureManager.getFigure('Vehicle Covariances');
clf
plot(results{1}.vehicleCovarianceHistory')
legend('x covariance', 'y covariance', 'theta covariance', 'Location', 'best')
title('Vehicle Covariances')
xlabel('Timestep')
ylabel('Covariance')
hold on
saveas(gcf,'Figures/3a/q3_a_covariances.png')

% Plot errors
minislam.graphics.FigureManager.getFigure('Errors');
clf
plot(results{1}.vehicleStateHistory'-results{1}.vehicleTrueStateHistory')
legend('x error', 'y error', 'theta error', 'Location', 'best')
title('Vehicle State Errors')
xlabel('Timestep')
ylabel('Error')
hold on
saveas(gcf,'Figures/3a/q3_a_errors.png')

% Plot chi2 values
minislam.graphics.FigureManager.getFigure('chi2');
clf
plot(results{1}.chi2Time, results{1}.chi2History)
hold on
legend('Chi^2 values', 'Location', 'best')
title('Chi2 Values')
xlabel('Timestep')
ylabel('Chi2')
saveas(gcf,'Figures/3a/q3_a_chi2.png')
