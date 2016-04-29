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
            covi_xyz = [10; 10; 10];
            covi_dxyz = [1; 1; 1];
            covi_theta = [1; 1; 1];
            covi_dtheta = [0.1; 0.1; 0.1];
            P = diag([covi_xyz; covi_dxyz; covi_theta; covi_dtheta]);
        end
        
        % state process function
        function snew = processFcn(obj, s, dt)
            snew = 0;
        end
        
        % state measurement function
        function y = measurementFcn(obj, s, meas)
           y = 0;
        end
        
        % request the next (filtered) measurement
        function m = getNextMeasurement(obj)
            m = [];
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

