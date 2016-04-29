classdef Node < handle

    properties
        id = 0;
        name = '';
        
        % true state variables
        true_p = [0;0;0];
        mobile = false;
        
        % corresponding rigid body id, if needed
        rigidbody_id = [];
        
        % state estimation variables
        state_p = [0;0;0];
        state_clkofst = 0;
        state_clkbias = 0;
        
        % --- process variances ---
        % by default a node is static with no var on position
        var_p = [0.0; 0.0; 0.0];       
        var_co = 10e-12; % was 1e-15
        %var_co = 1e-20;
        % clock bias experimentally drifts as much as +/- 2.0 ns / sec
        var_cb = 1.00; % ??????
        
        % --- initial variances ---
        % initial position variance, dependent on area size
        vari_p = [1; 1; 1];
        % clock offset is uniform [0,17] roughly, or 24 var
        vari_co = 1e-3;
        % initial bias could be as high as +/- 2ppm - +/- 2ppm = +/-4ppm (uniform)
        vari_cb = 1;
        
        % message transmission times
        tlast_type1 = 0;
        tlast_type2 = 0;
        tlast_type3 = 0;
        
        % reference node
        is_reference = false;
    end
    
    methods
        % Constructor
        function obj = Node(id, name)
            obj.id = id;
            obj.name = name;
        end
        
        % make this a mobile node
        function setToMobile(obj)
            obj.mobile = true;
            obj.var_p = [0.2; 0.2; 0.2];
        end
        
        % make this a static node
        function setToStatic(obj)
            obj.mobile = false;
        end
        
        % assign a rigid body id
        function setRigidBodyId(obj, rb)
            obj.rigidbody_id = rb;
        end
        
        % get rigid body id
        function rb = getRigidBodyId(obj)
            rb = obj.rigidbody_id;
        end
        
        % fix static location to true location
        function setPositionToTrue(obj)
            obj.state_p = obj.true_p;
            obj.vari_p = [0; 0; 0];
        end
        
        % is this node mobile?
        function m = isMobile(obj)
            m = obj.mobile;
        end
        
        % set true position
        function setTruePosition(obj, pos)
            if ~iscolumn(pos)
                pos = pos';
            end
            obj.true_p = pos;
        end
        
        % set the time a measurement of a given type was last sent
        function setLastMeasTime(obj, type, time)
            switch type
                case 1
                    obj.tlast_type1 = time;
                case 2
                    obj.tlast_type2 = time;
                case 3
                    obj.tlast_type3 = time;
                otherwise
                    error('Unrecognized measurement type');
            end
        end
        
        % get the time a measurement of a given type was last sent
        function t = getLastMeasTime(obj, type)
            switch type
                case 1
                    t = obj.tlast_type1;
                case 2
                    t = obj.tlast_type2;
                case 3
                    t = obj.tlast_type3;
                otherwise
                    error('Unrecognized measurement type');
            end
        end
        
        % get true position
        function p = getTruePosition(obj)
            p = obj.true_p;
        end
        
        % set the estimated state position
        function setStatePosition(obj, pos)
            if ~iscolumn(pos)
                pos = pos';
            end
            obj.state_p = pos;
        end
        
        % set the clock offset
        function setStateClockOfst(obj, ofst)
            obj.state_clkofst = ofst;
        end
        
        % set the clock bias
        function setStateClockBias(obj, bias)
            obj.state_clkbias = bias;
        end
        
        % get the estimated state position
        function p = getStatePosition(obj)
            p = obj.state_p;
        end
        
        % get the node id
        function id = getId(obj)
            id = obj.id;
        end
        
        % get the node name (string)
        function name = getName(obj)
            name = obj.name;
        end
        
        % get the state vector
        function s = getState(obj)
            s = [obj.state_p; obj.state_clkofst; obj.state_clkbias];
        end
        
        % set the state vector
        function setState(obj, si)
            obj.state_p = si(1:3);
            obj.state_clkofst = si(4);
            obj.state_clkbias = si(5);
        end

        % get the process variance vector
        function q = getProcessVar(obj)
            q = [obj.var_p; obj.var_co; obj.var_cb];
        end
        
        % get the initial variance vector
        function q = getInitialVar(obj)
            q = [obj.vari_p; obj.vari_co; obj.vari_cb];
        end
        
        % set this as the reference node
        function setAsReference(obj)
            obj.state_clkofst = 0;
            obj.state_clkbias = 0;
            obj.vari_p = [0; 0; 0];
            obj.var_co = 0;
            obj.vari_co = 0;
            obj.var_cb = 0;
            obj.vari_cb = 0;
            obj.is_reference = true;
            
        end
        
        % is this node a reference
        function val = isReference(obj)
            val = obj.is_reference;
        end
        
        % set covariances
        function setPositionCovar(obj, v)
            obj.var_p = [v; v; v];
        end
        
        function setOffsetCovar(obj, v)
            obj.var_co = v;
        end
        
        function setBiasCovar(obj, v)
            obj.var_cb = v;
        end
        
    end
    
end

