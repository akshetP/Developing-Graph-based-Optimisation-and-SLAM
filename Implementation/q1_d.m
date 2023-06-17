% This script runs Q1(d)

% Create the configuration object.
configuration = drivebot.SimulatorConfiguration();

% Only enable the GPS
configuration.enableGPS = true;

% If you set this parameter to false, the simulator generates measurements
% with no noise in them. You might find this useful for debugging.
% However, unless specified otherwise, any submitted results for any questions or subparts of questions
% must have this value set to true.
configuration.perturbWithNoise = true;

% Set up the simulator
simulator = drivebot.DriveBotSimulator(configuration, 'q1_d');

% Create the localization system
drivebotSLAMSystem = drivebot.DriveBotSLAMSystem(configuration);

% This tells the SLAM system to do a very detailed check that the input
% appears to be correct but can make the code run slowly. Once you are
% confident your code is working safely, you can set this to false.
drivebotSLAMSystem.setValidateGraph(true);

% Run the main loop and correct results
results = minislam.mainLoop(simulator, drivebotSLAMSystem);

% Minimal output plots. For your answers, please provide titles and label
% the axes.


%Make folder for plot if not exists
mkdir("Figures/1d_ii")

% Adding labels to the simulator output
title('Simulator Output')
xlabel('x position')
ylabel('y position')
saveas(gcf,'Figures/1d_ii/1.14 q1_d_sim_out.png')

% Plot optimisation times
minislam.graphics.FigureManager.getFigure('Optimization times');
clf
plot(results{1}.vehicleStateTime, results{1}.optimizationTimes, '*')
legend('Optimization Times', 'Location', 'best')
title('Optimization times')
xlabel('Timestep')
ylabel('Optimisation Time (sec)')
hold on


% Plot covariance
minislam.graphics.FigureManager.getFigure('Vehicle Covariances');
clf
plot(results{1}.vehicleStateTime, results{1}.vehicleCovarianceHistory')
legend('x covariance', 'y covariance', 'theta covariance', 'Location', 'best')
title('Vehicle Covariances')
xlabel('Timestep')
ylabel('Covariance')
hold on
saveas(gcf,'Figures/1d_ii/1.16 q1_d_covariances.png')

% Plot errors
minislam.graphics.FigureManager.getFigure('Errors');
clf
plot(results{1}.vehicleStateTime, results{1}.vehicleStateHistory'-results{1}.vehicleTrueStateHistory')
legend('x error', 'y error', 'theta error', 'Location', 'best')
title('Vehicle Error')
xlabel('Timestep')
ylabel('Error')
hold on
saveas(gcf,'Figures/1d_ii/1.15 q1_d_truestatehistoryerrors.png')