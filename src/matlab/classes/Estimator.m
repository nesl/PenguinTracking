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
        function P = getProcessVar(obj)
            P = obj.rbody.getProcessCovariance();
        end
        
        % state process function
        function snew = processFcn(obj, s, dt)

        end
        
        % state measurement function
        function y = measurementFcn(obj, s, meas)
           
        end
        
        % request the next (filtered) measurement
        function m = getNextMeasurement(obj)
            while obj.msgidx < obj.dataparser.getNumMeasurements()
                % get new tentative meas
                raw = obj.dataparser.getMeasurement(obj.msgidx);
                meas = Measurement(obj.MSGTYPE3, raw);
                obj.msgidx = obj.msgidx + 1;
                % check if we should pass this message
                if obj.filterMeasurement(meas) == obj.PASS
                    % ANT corrections
                    if ~isempty(obj.antennacorrections)
                        idx = find(obj.antennacorrections(:,1) == meas.getSourceId() &...
                            obj.owrcorrections(:,2) == meas.getDestId());
                        if ~isempty(idx)
                            meas.d_ij = meas.d_ij + 0*1e-9*obj.antennacorrections(idx, 3);
                        end
                    end
                    % OWR corrections
                    if ~isempty(obj.owrcorrections)
                        idx = find(obj.owrcorrections(:,1) == meas.getSourceId() &...
                            obj.owrcorrections(:,2) == meas.getDestId());
                        if ~isempty(idx)    
                            meas.r_ij = meas.r_ij + obj.owrcorrections(idx, 3);
                        end
                    end
                    % TWR corrections
                    if ~isempty(obj.twrcorrections)
                        idx = find(obj.twrcorrections(:,1) == meas.getSourceId() &...
                            obj.twrcorrections(:,2) == meas.getDestId());
                        if ~isempty(idx)    
                            meas.R_ij = meas.R_ij + obj.twrcorrections(idx, 3);
                        end
                    end
                    
                    m = meas;
                    
                    % increment and check message count
                    if obj.meascnt > obj.MAXMEASUREMENTS
                        m = [];
                        return;
                    end
                    obj.meascnt = obj.meascnt + 1;
                    obj.measurement_history{obj.meascnt} = m;
                    return;
                end
            end
            % no more measurements
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
        
        
        % =============================================
        %              MEASUREMENT HISTORY
        % =============================================
        
        % get pairwise TWR range measurements
        function r = getMeasurementTWR(obj, srcId, dstId)
            r = [];
            for i=1:obj.meascnt
                meas = obj.measurement_history{i};
                if meas.getSourceId() == srcId && meas.getDestId() == dstId
                    r = [r; meas.R_ij];
                end
            end
        end
        
        % get pairwise OWR range measurements
        function r = getMeasurementOWR(obj, srcId, dstId)
            r = [];
            for i=1:obj.meascnt
                meas = obj.measurement_history{i};
                if meas.getSourceId() == srcId && meas.getDestId() == dstId
                    r = [r; meas.r_ij];
                end
            end
        end
        
        % get pairwise offset measurements
        function [o,t] = getMeasurementOffsets(obj, srcId, dstId)
            o = [];
            for i=1:obj.meascnt
                meas = obj.measurement_history{i};
                if meas.getSourceId() == srcId && meas.getDestId() == dstId
                    o = [o; meas.d_ij];
                end
            end
        end
        
        % get all measurements between two devices
        function m = getMeasurements(obj, srcId, dstId)
            m = [];
            for i=1:obj.meascnt
                meas = obj.measurement_history{i};
                if meas.getSourceId() == srcId && meas.getDestId() == dstId
                    m = [m; meas];
                end
            end
        end
        
        % get pairwise measurement times
        function t = getMeasurementTimes(obj, srcId, dstId)
            t = [];
            for i=1:obj.meascnt
                meas = obj.measurement_history{i};
                if meas.getSourceId() == srcId && meas.getDestId() == dstId
                    t = [t; meas.getTime()];
                end
            end
        end
        
        % get all measurement times
        function t = getAllMeasurementTimes(obj)
            t = zeros(1,obj.meascnt);
            for i=1:obj.meascnt
                meas = obj.measurement_history{i};
                t(i) = meas.getTime();
            end
        end
        
        % get allan deviation between two nodes
        function [ad,tau] = getAllanDev(obj, srcId, dstId, tau)
            % get non-zero aligned data
            data = obj.dataparser.aligned_logs;
            data = data(data(:,2) == srcId & data(:,3) == dstId, :);
            % get tx and rx
            tx = data(:,5);
            rx = data(:,6);
            % tx time (ignore 1st b/c y is diffed)
            x = tx(2:end);
            % rx dFreq in seconds
            y = (diff(rx) ./ diff(tx) - 1);
            [ad,~,~,tau] = allan(struct('freq',y,'time',x),tau,'',0);
        end
        
        % get all time offsets between two nodes
        function [o,t] = getAllOffsets(obj, srcId, dstId)
            data = obj.dataparser.aligned_logs;
            idxs = find(data(:,2) == srcId & data(:,3) == dstId);
            t = data(idxs,1);
            o = data(idxs,10) - data(idxs,9);
        end
        
        % get all time biases between two nodes
        function [b,t] = getAllBiases(obj, srcId, dstId)
            data = obj.dataparser.aligned_logs;
            idxs = find(data(:,2) == srcId & data(:,3) == dstId);
            t = data(idxs,1);
            o = data(idxs,10) - data(idxs,9);
            b = diff(o)./diff(t);
            t = t(2:end);
        end
        
        % =============================================
        %              POSITION ACCESSORS
        % =============================================
        function xyz = getTruePosition(obj, nodeId, tarray)
            xyz = [];
            for i=1:length(tarray)
                t = tarray(i);
            idx = obj.getNodeIdx( nodeId );
            if obj.nodes{idx}.isMobile()
                rbid = obj.nodes{idx}.getRigidBodyId();
                p = obj.dataparser.getMocapPos( rbid, t );
            else
                p = obj.nodes{ idx }.getTruePosition()';
            end
            xyz = [xyz; p];
            end
        end
        
        function setRigidBodyId(obj, nodeId, rbId)
            nodeIdx = obj.getNodeIdx( nodeId );
            obj.nodes{nodeIdx}.setRigidBodyId( rbId );
        end
        
    end
    
end

