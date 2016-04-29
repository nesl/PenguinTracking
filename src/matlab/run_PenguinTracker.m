%% Clean up console and variables
clc; close all; clear all;
addpath('classes');
addpath('utilities');

%% Raw Data Log Files
fp_inertial = '../../data/inertial_01.csv';
fp_gps = '../../data/gps_01.csv';

%% Create or load dataparser
dp = DataParser( fp_inertial, fp_gps );
% advance data to first GPS fix
dp.advanceToFirstFix();

%% Create rigid body estimator
rbody = RigidBody('Penguin', [0;0;0], [0;0;0]);
estimator = Estimator(dp, rbody);

%% Process Covariances
Q = estimator.getProcessVar();
P = estimator.getInitialVar();

%% Run EKF
% stop time
t_stop = 60*60;

% state and time history
s_history = [];
t_history = [];

% get the very first measurement
meas1 = dp.getNextMeasurement();
t_start = meas1.getTime();
t_last = t_start;
k = 0;

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
    
    
    % delta-time uses wallclock (desktop timestamp) for now
    dt = meastime - t_last;
    t_last = meastime;
    
    % get network state vector
    s = estimator.getState();
    
    % configure process and measurement functions
    f = @(s) estimator.processFcn(s, dt);
    h = @(s) estimator.measurementFcn(s, meas);
    
    % update state estimate
    [s, P] = ekf(f, s, P, h, z, dt*Q, R);
    
    % update network state vector
    estimator.setState(s);
    
    % append state estimate & measurement to history
    s_history = [s_history; s'];

end

% save data
save('cache/estimate_01', 'estimator', 'dp', 's_history', 't_history');

return;


