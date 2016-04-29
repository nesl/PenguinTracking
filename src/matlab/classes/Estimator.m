classdef Estimator < handle
    %NETWORKMANAGER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Constant)
        PLTSTYLES = {'-o', '-s', '-^', '-x', '-*', '--o', '--s', '--^', '--x', '--*'};
        PLTCOLORS = [
            1 0 0
            0 0 1
            0 0 0
            0.8 0.8 0
            1 0 1
            0 0.7 0.7
            0 1 0
            0.5 0.5 0.5
            0.25 0.25 1
            1 0.25 0.25
            0.25 1 0.25
            ];
        PASS = 1;
        BLOCK = 0;
        MAXMEASUREMENTS = 1e6;
        MSGTYPE_INERTIAL = 0;
        MSGTYPE_GPS = 1;
        GRAVITY = 9.81;
    end
    
    properties
        % rigid body (e.g. Penguin)
        rbody = [];

        % message / data variables
        dataparser = [];
        msgidx = 1;
        measurement_history = {};
        meascnt = 0;
        
    end
    
    methods
        % =============================================
        %                  CONSTRUCTOR
        % =============================================
        function obj = Estimator(dataparser, rbody)
            % create dataparser
            obj.dataparser = dataparser;
            obj.rbody = rbody;            
            
            % pre-allocate measurement memory
            obj.measurement_history = cell(obj.MAXMEASUREMENTS,1);
        end
        
        % =============================================
        %                  PLOTTING
        % =============================================
        
        % plot styles and colors
        function style = getPlotStyle(obj, idx)
            style = obj.PLTSTYLES{idx};
        end
        function c = getPlotColor(obj, idx)
            c = obj.PLTCOLORS(idx,:);
        end
        
        % =============================================
        %                  RBODY INFO
        % =============================================
        
        % get the rigid body
        function rb = getRigidBody(obj)
            rb = obj.rbody;
        end
        
        % =============================================
        %               STATE ACCESSORS
        % =============================================
        
        % get entire state
        function s = getState(obj)
            s = obj.rbody.getStateVector();
        end

        % set entire state
        function setState(obj, s)
            obj.rbody.setStateVector(s);
        end
        
        % get estimated position
        function xyz = getPosition(obj)
            xyz = obj.rbody.getPosition();
        end
        
        % get estimated orientation
        function theta = getOrientation(obj)
            theta = obj.rbody.getOrientation();
        end
        
        % =============================================
        %               PROCESS & MEASUREMENT
        % =============================================
        
        % get process variance
        function Q = getProcessVar(obj)
            Q = obj.rbody.getProcessCovariance();
        end
        
        % initial process variance
        function P = getInitialVar(obj)
            covi_xyz = 1e3*[20; 20; 20];
            covi_dxyz = [1; 1; 1];
            covi_ddxyz = [1; 1; 1];
            covi_theta = [1; 1; 1];
            P = diag([covi_xyz; covi_dxyz; covi_ddxyz; covi_theta]);
        end
        
        % state process function
        function snew = processFcn(obj, s, dt)
            % to start, predicted new state is old state
            snew = s;
            
            % xyz, dxyz, theta, dtheta
            xyz = s(1:3);
            dxyz = s(4:6);
            ddxyz = s(7:9);
            theta = s(10:12);
            
            % simple integration of velocities
            snew(1:3) = xyz + dt*dxyz;
            snew(4:6) = dxyz + dt*ddxyz;
        end
        
        % state measurement function
        function y = measurementFcn(obj, s, meas)
            if meas.getType() == obj.MSGTYPE_INERTIAL
                theta = s(4:6);
                % rotation matrices from world to body frame
                Rx = [1 0 0; 0 cos(theta(1)) -sin(theta(1)); 0 sin(theta(1)) cos(theta(1))];
                Ry = [cos(theta(2)) 0 sin(theta(2)); 0 1 0; -sin(theta(2)) 0 cos(theta(2))];
                Rz = [cos(theta(3)) -sin(theta(3)) 0; sin(theta(3)) cos(theta(3)) 0; 0 0 1];
                R = Rz*Ry*Rx;
                % acc_xyz, mag_xyz, temp, baro (body frame!)
                state_acc = s(1:3);
                state_mag = s(4:6);
                y = [
                    % acc_xyz
                    R*(state_acc);
                    % mag_xyz
                    Rx*[1;1;1];
                    % temp
                    0;
                    % baro
                    0;
                    ];
                
            elseif meas.getType() == obj.MSGTYPE_GPS
                % x_utm, y_utm, alt
                y = [
                    % x
                    s(1);
                    % y
                    s(2);
                    % alt
                    s(3);
                    ];
            else
                error('Unsupported message type');
            end
        end
        
        % request the next (filtered) measurement
        function m = getNextMeasurement(obj)
            m = obj.dataparser.getNextMeasurement();
        end
        
        function killLastMeasurement(obj)
            obj.meascnt = obj.meascnt - 1;
        end
        
        % reset measurement index
        function resetMeasurements(obj)
            obj.msgidx = 1;
            obj.measurement_history = cell(obj.MAXMEASUREMENTS,1);
            obj.meascnt = 0;
        end
        
    end
    
end

