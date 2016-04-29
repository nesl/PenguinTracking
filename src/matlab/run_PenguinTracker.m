%% Clean up console and variables
clc; close all; clear all;
addpath('classes');
addpath('utilities');

%% Raw Data Log Folder
logfolder = '~/Dropbox/Data/Data_SLATS/ros_mocap/good/ped04/';
%logfolder = '~/Desktop/logs/test/';

%% Node/Network configuration
configfile = 'config/nodepositions_nesl_mobile';

%% Create Network Manager
% NetworkManager(nodeconfig, datafolder, <owr_corrections>, <twr_corrections>) 
nm = NetworkManager(configfile, logfolder, 'config/antennacorrections_mocap', 'config/twrcorrections_mocap_new', 'config/twrcorrections_mocap_new' );
node_ids = nm.getNodeIds();
node_names = nm.getNodeNames();

%% Rigid body IDs
nm.setRigidBodyId('ntb-mobile', 1);

%% Black-listed nodes
%nm.blacklistNode('ntb-charlie');

%% Select reference node
nm.setReferenceNode('ntb-alpha');

%% Bootstrap the node positions and clock params before filter starts
% use first second of measurements to bootstrap node states
%nm.bootstrapNodeStates( 2.0 );
%nm.setStaticNodesToTruePosition();

%% Measurement set up
%{
% R_ij ONLY
nm.enableMessageType(nm.MSGTYPE3, true);
nm.enableMeasurementType(nm.MEASTYPE_R, true);
nm.enableSLATS(false);
for nidx=1:length(node_ids)
    if nm.nodes{nidx}.isMobile()
        nm.nodes{nidx}.setPositionCovar(0.50);
        nm.nodes{nidx}.setOffsetCovar(10e-12);
        nm.nodes{nidx}.setBiasCovar(1.00);
    else
        nm.nodes{nidx}.setPositionCovar(0.00);
        nm.nodes{nidx}.setOffsetCovar(10e-12);
        nm.nodes{nidx}.setBiasCovar(1.00);
    end
end

% MEASUREMENT COVAR:
%        var_dij = 1e-9; 
%        var_rij = 1.00;   
%        var_Rij = 0.30;

%}

%{
% r_ij only
nm.enableMessageType(nm.MSGTYPE3, true);
nm.enableMeasurementType(nm.MEASTYPE_r, true);
nm.enableMeasurementType(nm.MEASTYPE_d, true);
nm.enableSLATS(false);
for nidx=1:length(node_ids)
    if nm.nodes{nidx}.isReference()
        continue;
    end
    if nm.nodes{nidx}.isMobile()
        nm.nodes{nidx}.setPositionCovar(0.50);
        nm.nodes{nidx}.setOffsetCovar(10e-9);
        nm.nodes{nidx}.setBiasCovar(10.00);
    else
        nm.nodes{nidx}.setPositionCovar(0.01);
        nm.nodes{nidx}.setOffsetCovar(10e-9);
        nm.nodes{nidx}.setBiasCovar(10.00);
    end
end
% MEASUREMENT COVAR:    
%   var_dij = 10e-9;
%   var_rij = 1.00;   
%   var_Rij = 0.30;
%}

%%{
% d_ij only
nm.enableMessageType(nm.MSGTYPE3, true);
nm.enableMeasurementType(nm.MEASTYPE_d, true);
nm.enableSLATS(true);
for nidx=1:length(node_ids)
    if nm.nodes{nidx}.isReference()
        continue;
    end
    if nm.nodes{nidx}.isMobile()
        nm.nodes{nidx}.setPositionCovar(0.30);
        nm.nodes{nidx}.setOffsetCovar(1e-18);
        nm.nodes{nidx}.setBiasCovar(10.00);
    else
        nm.nodes{nidx}.setPositionCovar(0.01);
        nm.nodes{nidx}.setOffsetCovar(1e-18);
        nm.nodes{nidx}.setBiasCovar(10.00);
    end
end
% MEASUREMENT COVAR:
%   var_dij = 10e-12;
%   var_rij = 1.00;   
%   var_Rij = 0.30;
%%}

%% Set some known anchor positions if desired
known_anchors = {'ntb-alpha', 'ntb-bravo', 'ntb-charlie', 'ntb-delta', 'ntb-hotel', 'ntb-golf', 'ntb-echo', 'ntb-foxtrot'};
%known_anchors = {};
for i=1:length(known_anchors)
    aid = known_anchors{i};
    aidx = nm.getNodeIdx( aid );
    nm.nodes{ aidx }.var_p = [0.0; 0.0; 0.0];
    nm.nodes{ aidx }.vari_p = [0.0; 0.0; 0.0];

end

%% Process Covariances
% Process and Initial noise and state vals are determined by each node object
Q = nm.getProcessVar();
P = nm.getInitialVar();
% minimum P values for stability
pmin_p = [0.001; 0.001; 0.001];
pmin_o = 1e-15;
pmin_b = 0.1;
Pmin_vec = [0;0;0;0;0];
for i=2:length(node_ids)
    Pmin_vec = [Pmin_vec; [pmin_p; pmin_o; pmin_b]];
end
Pmin = diag(Pmin_vec);


%% Save as movie
SAVEMOVIE = false;
if SAVEMOVIE
    vidObj = VideoWriter('output/mocap.avi');
    vidObj.FrameRate=20;
    open(vidObj);
end

%% Position Visualization
% get current true and estimated positions
pTruStatic = nm.getTrueStaticPositions();
pEstAll = nm.getTransformedPositions();

fig = cfigure(25,25); grid on; hold on; axis equal;

htruStatic = plot3(pTruStatic(:,2),pTruStatic(:,3),pTruStatic(:,4),'^k','MarkerSize',10, 'MarkerFaceColor', 'k');
hestAll = plot3(pEstAll(:,1),pEstAll(:,2),pEstAll(:,3),'r+');
herrStatic = zeros(nm.getNumStaticNodes(),1);
hvar = zeros(nm.getNumNodes(),1);
varscale = 1.00;
for i=1:nm.getNumNodes()
    nid = node_ids(i);
    % if node is static, plot error line to true position
    if ~nm.nodes{i}.isMobile()
        herrStatic(i) = plot3([pTruStatic(i,2) pEstAll(i,1)],[pTruStatic(i,3) pEstAll(i,2)],[pTruStatic(i,4) pEstAll(i,3)], nm.getPlotStyle(nid), 'Color', nm.getPlotColor(nid));
    end
    % if node is the reference, add text saying so
    if nm.nodes{i}.isReference()
        xyz = nm.nodes{i}.getTruePosition();
        text( xyz(1)-0.50, xyz(2) - 0.30, xyz(3), 'Reference');
    end
    % add text for each node
    if ~nm.nodes{i}.isMobile()
        xyz = nm.nodes{i}.getTruePosition();
        text( xyz(1)-0.50, xyz(2) + 0.50, xyz(3), nm.nodes{i}.getName() );
    end
    
    % get a node's position covariance matrix    
    nidx = (i-1)*5 + 1;
    Pi = P( nidx:(nidx+2), nidx:(nidx+2) ) + 0.01*[1 0 0; 0 0 0; 0 0 1];
    hvar(i) = plotEllipse([pEstAll(i,1); pEstAll(i,2); pEstAll(i,3)], Pi, varscale);
end

% rigid bodies in mocap
rigid_bodies = nm.dataparser.getRigidBodyIds();
hrigidbodies = zeros( length(rigid_bodies),1 );
for i=1:length(rigid_bodies)
    hrigidbodies(i) = plot3(0, 0, 0, 'sb', 'MarkerSize', 10, 'MarkerFaceColor', 'k', 'LineWidth', 2);
end

xlim([-7 7]);
ylim([0 4]);
zlim([-7 7]);
xlabel('X Position (m)', 'FontSize',14);
ylabel('Y Position (m)', 'FontSize',14);
zlabel('Z Position (m)', 'FontSize',14);
htitle = title('NESL Network Localization (t = 0.00s)','FontSize',16);
view(180,0);
%legend(herr, nm.getNodeNames());
drawnow;

%% Replay data and run EKF
% analysis stop time
t_stop = 10;
% state, time, and transformed position history
s_history = [];
p_history = {};
rangeerr_slats = [];
rangeerr_none = [];


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


