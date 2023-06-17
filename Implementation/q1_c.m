% This script runs Q1(c)

% Create the configuration object.
configuration = drivebot.SimulatorConfiguration();

% Q1c: Set the configuration to enable the compass
configuration.enableCompass = true;

% Set the compass angular offset. DO NOT change this value.
configuration.compassAngularOffset=0.75*pi;

% If you set this parameter to false, the simulator generates measurements
% with no noise in them. You might find this useful for debugging.
% However, unless specified otherwise, any submitted results must have this
% value set to true.
configuration.perturbWithNoise = true;

% Set up the simulator
simulator = drivebot.DriveBotSimulator(configuration, 'q1_c');

% Create the localization system
drivebotSLAMSystem = drivebot.DriveBotSLAMSystem(configuration);

% Force the optimizer to run with this frequency. This lets you see what's
% happening in greater detail, but slows everything down.
drivebotSLAMSystem.setRecommendOptimizationPeriod(20);

% This tells the SLAM system to do a very detailed check that the input
% appears to be correct but can make the code run slowly. Once you are
% confident your code is working safely, you can set this to false.
drivebotSLAMSystem.setValidateGraph(true);

% Run the main loop and correct results
results = minislam.mainLoop(simulator, drivebotSLAMSystem);

% Minimal output plots. For your answers, please provide titles and label
% the axes.

%Make folder for plot if not exists
mkdir("Figures/1c_i_buggy")
mkdir("Figures/1c_ii_fixed")


% Adding labels to the simulator output
title('Simulator Output')
xlabel('x position')
ylabel('y position')
%saveas(gcf,'Figures/1c_i_buggy/1.6 q1_c_sim_out.png')
saveas(gcf,'Figures/1c_ii_fixed/1.101 q1_c_sim_out.png')


% Plot covariance
minislam.graphics.FigureManager.getFigure('Vehicle Covariances');
clf
plot(results{1}.vehicleStateTime, results{1}.vehicleCovarianceHistory')
legend('x covariance', 'y covariance', 'theta covariance', 'Location', 'best')
title('Vehicle Covariances')
xlabel('Timestep')
ylabel('Covariance')
hold on
%saveas(gcf,'Figures/1c_i_buggy/1.8 q1_c_covariances.png')
saveas(gcf,'Figures/1c_ii_fixed/1.11 q1_c_covariances.png')

% Plot errors
minislam.graphics.FigureManager.getFigure('Errors');
clf
plot(results{1}.vehicleStateTime, results{1}.vehicleStateHistory'-results{1}.vehicleTrueStateHistory')
legend('x error', 'y error', 'theta error', 'Location', 'best')
title('Vehicle Error')
xlabel('Timestep')
ylabel('Error')
hold on
%saveas(gcf,'Figures/1c_i_buggy/1.9 q1_c_truestatehistoryerrors.png')
saveas(gcf,'Figures/1c_ii_fixed/1.12 q1_c_truestatehistoryerrors.png')

% Plot chi2 values
minislam.graphics.FigureManager.getFigure('chi2');
clf
plot(results{1}.chi2Time, results{1}.chi2History)
hold on
legend('Chi^2 values', 'Location', 'best')
title('Chi2 Values')
xlabel('Timestep')
ylabel('Chi2')
%saveas(gcf, 'Figures/1c_i_buggy/1.7 q1_c_chi2.png')
saveas(gcf, 'Figures/1c_ii_fixed/1.10 q1_c_chi2.png')

% Plot optimisation times
minislam.graphics.FigureManager.getFigure('Optimization times');
clf
plot(results{1}.vehicleStateTime, results{1}.optimizationTimes, '*')
legend('Optimization Times', 'Location', 'best')
title('Optimization times')
xlabel('Timestep')
ylabel('Optimisation Time (sec)')
hold on
%saveas(gcf, 'Figures/1c_i_buggy/q1_c_optim.png')
saveas(gcf, 'Figures/1c_ii_fixed/q1_c_optim.png')
