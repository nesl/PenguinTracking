%% Clean up console and variables
clc; close all; clear all;
addpath('classes');
addpath('utilities');

%% Raw Data Log Files
fp_inertial = '../../data/inertial_01.csv';
fp_gps = '../../data/gps_01.csv';

%% Create or load dataparser
fprintf('Loading data...\n');
dp = DataParser( fp_inertial, fp_gps );
% advance data to first GPS fix
dp.advanceToFirstFix();

%% Create rigid body estimator
fprintf('Creating rigid body...\n');
rbody = RigidBody('Penguin', [0;0;0], [0;0;0]);
fprintf('Creating estimator...\n');
estimator = Estimator(dp, rbody);

%% Process Covariances
Q = estimator.getProcessVar();
P = estimator.getInitialVar();

%% Run EKF
% stop time
t_stop = 24*3600;

% state, measurement, and time history
s_history = [];
m_history = [];
t_history = [];

% get the very first measurement
meas1 = dp.getNextMeasurement();
t_start = meas1.getTime();
t_last = t_start;
k = 0;

fprintf('Running EKF...\n');
while (t_last - t_start) < t_stop
    k = k + 1;
    
    % get next measurement object
    meas = dp.getNextMeasurement();
    if isempty(meas)
        break;
    end
    meastime = meas.getTime();
    z = meas.vectorize();
    R = meas.getCovariance();     
        
    % delta-time for process model
    dt = meastime - t_last;
    t_last = meastime;
    
    % get network state vector
    s = estimator.getState();
    
    % configure process and measurement functions
    f = @(s) estimator.processFcn(s, dt);
    h = @(s) estimator.measurementFcn(s, meas);
    
    % update state estimate
    [s, P] = ekf(f, s, P, h, z, dt*Q, R);
    
    % slight cheat -- zero-velocity update (ZUPT) on GPS data:
    if meas.getType() == 1
        s(4:6) = [0;0;0];
        s(7:9) = [0;0;0];
    end
        
    % update network state vector
    estimator.setState(s);
    
    % append state estimate & measurement to history
    s_history = [s_history; s'];
    m_history = [m_history; meas];
    t_history = [t_history; t_last];
    
    fprintf('Estimate: (%.5f, %.5f, %.5f) t = %.2f min.\n', s(1), s(2), s(3), (t_last-t_start)/60);
    %pause(0.10);

end

% save data
save('cache/estimate_01', 'estimator', 'dp', 's_history', 't_history');

%% Display
load('cache/estimate_01');
plot(s_history(2:end,1), s_history(2:end,2), 'bo');
xlabel('X');
ylabel('Y');
zlabel('Z');
grid on;



