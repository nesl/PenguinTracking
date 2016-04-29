%% Clean up console and variables
clc; close all; clear all;
addpath('classes');
addpath('utilities');

%% Raw Data Log Files
fp_inertial = '~/Downloads/Dinodave swim_selection_1.txt';
fp_gps = '~/Downloads/Julian1_20151212-143221.csv';

%% Create or load dataparser
if ~exist('cache/last_dataparser.mat', 'file')
    disp('Data not previously loaded, loading now...');
    dp = DataParser( fp_inertial, fp_gps );
    save('cache/last_dataparser.mat', 'dp');
else
    disp('Loading pre-existing data cache...');
    load('cache/last_dataparser.mat');
end

%% Create rigid body estimator
rbody = RigidBody('Penguin', [0;0;0], [0;0;0]);
estimator = Estimator(dp, rbody);

%% Process Covariances
Q = estimator.getProcessVar();
P = estimator.getInitialVar();

%% Run EKF
% analysis stop time
t_stop = 3600;
% state, time, and transformed position history
s_history = [];
t_history = [];

return;

% last global time update
meas1 = nm.getNextMeasurement();
t_start = meas1.getTime();
t_last = t_start;
k = 0;

% plotting
plot_delay = 1*0.20; % sec
plot_last = t_start;
dispt_last = t_start;

tlast_twr = 0;
period_twr = 5.00;

meas_last = meas1;

while (t_last - t_start) < t_stop
    k = k + 1;
    
    % get next measurement object
    meas = nm.getNextMeasurement();
    if isempty(meas)
        break;
    end
    srcId = meas.getSourceId();
    dstId = meas.getDestId();
    srcIdx = nm.getNodeIdx(srcId);
    dstIdx = nm.getNodeIdx(dstId);
    walltime = meas.getTime();
    z = meas.vectorize();
    R = meas.getCovariance();  
    
    
    measClean = meas_last.getSourceId() == 7 || meas_last.getSourceId() == 8;
    if meas.getSourceId() == 8 && ~measClean
        nm.killLastMeasurement();
        continue;
    end
    
    xyz_src = nm.getTruePosition( srcId, walltime );
    xyz_dst = nm.getTruePosition( dstId, walltime );
    range_tru = sqrt( sum( (xyz_src - xyz_dst).^2, 2) );
    bi = nm.nodes{srcIdx}.state_clkbias;
    bj = nm.nodes{dstIdx}.state_clkbias;
    range_est = meas.r_ij - (nm.LIGHTSPEED*meas.T_rsp1/2)*(bj - bi)*1e-9;
    rangeerr_slats = [rangeerr_slats; (range_est - range_tru)];
    rangeerr_none = [rangeerr_none; (meas.r_ij - range_tru)];
    
    %if meas_last.getSourceId() ~= 8 && meas.getSourceId() == 8
    %    fprintf('meas_last = %d\n', meas_last.getSourceId());
    %end
    meas_last = meas;
    
    % delta-time uses wallclock (desktop timestamp) for now
    dt_ref = meas.getTime() - t_last;
    t_last = walltime;
    
    if walltime - dispt_last > 1.00
        fprintf('Time: %.2f (%.2f%%)\n', (walltime - t_start), 100*(walltime - t_start)/t_stop);
        dispt_last = walltime;
    end
    
    % Hack: throw out timing info that diverges too much
    %{
    ofst_i_last = nm.nodes{ nm.getNodeIdx(meas.getSourceId()) }.state_clkofst;
    bias_i_last = nm.nodes{ nm.getNodeIdx(meas.getSourceId()) }.state_clkbias;
    ofst_j_last = nm.nodes{ nm.getNodeIdx(meas.getDestId()) }.state_clkofst;
    bias_j_last = nm.nodes{ nm.getNodeIdx(meas.getDestId()) }.state_clkbias;
    dij_last = ofst_j_last - ofst_i_last;
    bij_last = bias_j_last - bias_i_last;
    dij_pred = dij_last + dt_ref*bij_last/1.0e9;
    delta = meas.d_ij - dij_pred;
    %fprintf('[%d->%d] delta: %.1f\n', meas.getSourceId(), meas.getDestId(), delta*1e9);
    if delta > 40e-9
        continue;
    end
    %}
    
    % get network state vector
    s = nm.getState();
    
    % configure process and measurement functions
    f = @(s) nm.processFcn(s, dt_ref);
    h = @(s) nm.measurementFcn(s, meas);
    
    % update state estimate
    [s, P] = ekf(f, s, P, h, z, dt_ref*Q, R);
    
    % update network state vector
    nm.setState(s);
    
    % covariance matrix stability
    %P = P + Pmin;
    
    % update position estimates
    pTruStatic = nm.getTrueStaticPositions();
    pEstAll = nm.getTransformedPositions();
    set(hestAll,'xdata',pEstAll(:,1),'ydata', pEstAll(:,2),'zdata',pEstAll(:,3));
    for i=1:nm.getNumNodes()
        if ~nm.nodes{i}.isMobile()
            set(herrStatic(i),'xdata',[pTruStatic(i,2) pEstAll(i,1)],'ydata',...
                [pTruStatic(i,3) pEstAll(i,2)],'zdata',[pTruStatic(i,4) pEstAll(i,3)]);
        end
        nidx = (i-1)*5 + 1;
        Pi = P( nidx:(nidx+2), nidx:(nidx+2) );
        updateEllipse( hvar(i), [pEstAll(i,1); pEstAll(i,2); pEstAll(i,3)], Pi + 0.001*[1 0 0; 0 0 0; 0 0 1], varscale);
        if i == 3
            % Pi
        end
    end    
    
    if walltime - plot_last >= plot_delay
        plot_last = walltime;
        % update rigid bodies
        for i=1:length(rigid_bodies)
            rb = rigid_bodies(i);
            [xyz, latency] = nm.dataparser.getMocapPos(rb, walltime);
            if latency < 0.250
                set(hrigidbodies(i), 'XData', xyz(1), 'YData', xyz(2), 'ZData', xyz(3));
            end
        end
        
        % update plot title
        tstr = sprintf('NESL Network Localization (t = %.2fs)', (t_last - t_start));
        set(htitle, 'String', tstr);
        drawnow;
    end
    
    % append state estimate & measurement to history
    s_history = [s_history; s'];
    p_history = [p_history; pEstAll];
    
    if SAVEMOVIE
        f = getframe(fig);
        writeVideo(vidObj,f);
    end
    
end

if SAVEMOVIE
    close(vidObj);
end

% save data
save('cache/slats_d_short', 'nm', 'k', 's_history', 'p_history','rangeerr_slats', 'rangeerr_none');

return;


