% This script runs Q1(b)

% Create the configuration object.
configuration = drivebot.SimulatorConfiguration();

% Since we are doing just prediction, all the other sensors are disabled.
% This is the default setting.

% If you set this parameter to false, the simulator generates measurements
% with no noise in them. You might find this useful for debugging.
% However, unless specified otherwise, any submitted results must have this
% value set to true.
configuration.perturbWithNoise = true;

% Set up the simulator
simulator = drivebot.DriveBotSimulator(configuration, 'q1_b');

% Create the localization system
drivebotSLAMSystem = drivebot.DriveBotSLAMSystem(configuration);

% This tells the SLAM system to do a very detailed check that the input
% appears to be correct but can make the code run slowly. Once you are
% confident your code is working safely, you can set this to false.
drivebotSLAMSystem.setValidateGraph(true);

% Run the main loop and correct results
results = minislam.mainLoop(simulator, drivebotSLAMSystem);

% Minimal output plots. These just show you an example of how to plot the
% results. For your report, you need to look at improving these figures
% including labelling axes, etc.


%Make folder for plot if not exists
mkdir("Figures/1b_ii")

% Adding labels to the simulator output
title('Simulator Output')
xlabel('x position')
ylabel('y position')
saveas(gcf,'Figures/1b_ii/1.5 q1_b_sim_out.png')

% Plot optimisation times
minislam.graphics.FigureManager.getFigure('Optimization times');
clf
plot(results{1}.vehicleStateTime, results{1}.optimizationTimes, '*')
legend('Optimsation Time', 'Location', 'best')
title('Optimization times')
xlabel('Timestep')
ylabel('Optimisation Time (sec)')
hold on
saveas(gcf,'Figures/1b_ii/1.4 q1_b_opt_times.png')

% Plot the error curves
minislam.graphics.FigureManager.getFigure('Errors');
clf
plot(results{1}.vehicleStateTime, results{1}.vehicleStateHistory'-results{1}.vehicleTrueStateHistory')
legend('x error', 'y error', 'theta error', 'Location', 'best')
title('Vehicle State Errors')
xlabel('Timestep')
ylabel('Error')
hold on
saveas(gcf,'Figures/1b_ii/1.2 q1_b_errors.png')

% Plot covariance
minislam.graphics.FigureManager.getFigure('Vehicle Covariances');
clf
plot(results{1}.vehicleStateTime, results{1}.vehicleCovarianceHistory')
legend('x covariance', 'y covariance', 'theta covariance', 'Location', 'best')
title('Vehicle Covariances')
xlabel('Timestep')
ylabel('Covariance')
hold on
saveas(gcf,'Figures/1b_ii/1.3 q1_b_covariances.png')