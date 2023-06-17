% This script runs Q1(e)

% Create the configuration object.
configuration = drivebot.SimulatorConfiguration();

% Since we are doing prediction and GPS, disable the SLAM sensor
configuration.enableGPS = true;

% Set to true for part ii
configuration.enableCompass = true;

% Set up the simulator
simulator = drivebot.DriveBotSimulator(configuration, 'q1_e');

% Create the localization system
drivebotSLAMSystem = drivebot.DriveBotSLAMSystem(configuration);

% Q1(e)i:
% Use the method "setRecommendOptimizationPeriod" in DriveBotSLAMSystem
% to control the rate at which the optimizer runs
drivebotSLAMSystem.setRecommendOptimizationPeriod(1); 
% This tells the SLAM system to do a very detailed check that the input
% appears to be correct but can make the code run slowly. Once you are
% confident your code is working safely, you can set this to false.
drivebotSLAMSystem.setValidateGraph(true);

% Run the main loop and correct results
results = minislam.mainLoop(simulator, drivebotSLAMSystem);

% Minimal output plots. For your answers, please provide titles and label
% the axes.

%Make folder for plot if not exists
mkdir("Figures/1e_i")
mkdir("Figures/1e_ii")


% Adding labels to the simulator output
title('Simulator Output')
xlabel('x position')
ylabel('y position')
if (configuration.enableCompass == false)
    saveas(gcf,'Figures/1e_i/1.17 q1_e_sim_out.png')
else 
    saveas(gcf,'Figures/1e_ii/1.20 q1_e_sim_out.png')
end



% Plot optimisation times
minislam.graphics.FigureManager.getFigure('Optimization times');
clf
plot(results{1}.vehicleStateTime, results{1}.optimizationTimes, '*')
legend('Optimization Times', 'Location', 'best')
title('Optimization times')
xlabel('Timestep')
ylabel('Optimisation Time (sec)')
hold on
if (configuration.enableCompass == false)
    saveas(gcf,'Figures/1e_i/1.19 q1_e_opt_times.png')
else
    saveas(gcf,'Figures/1e_ii/1.22 q1_e_opt_times.png')
end


% Plot covariance
minislam.graphics.FigureManager.getFigure('Vehicle Covariances');
clf
plot(results{1}.vehicleStateTime, results{1}.vehicleCovarianceHistory')
legend('x covariance', 'y covariance', 'theta covariance', 'Location', 'best')
title('Vehicle Covariances')
xlabel('Timestep')
ylabel('Covariance')
hold on

% Plot errors
minislam.graphics.FigureManager.getFigure('Errors');
clf
plot(results{1}.vehicleStateTime, results{1}.vehicleStateHistory'-results{1}.vehicleTrueStateHistory')
legend('x error', 'y error', 'theta error', 'Location', 'best')
title('Vehicle State Errors')
xlabel('Timestep')
ylabel('Error')
hold on



% Plot chi2 values
minislam.graphics.FigureManager.getFigure('chi2');
clf
plot(results{1}.chi2Time, results{1}.chi2History)
hold on
legend('Chi^2 values', 'Location', 'best')
title('Chi2 Values')
xlabel('Timestep')
ylabel('Chi2')
if (configuration.enableCompass == false)
    saveas(gcf, 'Figures/1e_i/1.18 q1_e_chi2.png')
else
    saveas(gcf, 'Figures/1e_ii/1.21 q1_e_chi2.png')
end


