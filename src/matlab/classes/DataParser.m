classdef DataParser < handle
    %DATAPARSER parse tcp data logs from NTB nodes
    %   obj = DataParser( config_file, log_folder )
    
    
    properties
        MSGTYPE_INERTIAL = 0;
        MSGTYPE_GPS = 1;
        data_inertial = {};
        data_gps = {}; 
        messages = [];
        % most recent measurements
        idx_inertial = 1;
        idx_gps = 1;
    end
    
    methods
        % Constructor
        function obj = DataParser( fp_inertial, fp_gps )
            % read inertial file
            obj.data_inertial = csvread(fp_inertial);
            
            % read GPS file
            obj.data_gps = csvread(fp_gps);
        end
        
        % get the next measurement, ordered in time
        function m = getNextMeasurement(obj)
            % is the next measurement inertial or GPS?
            if obj.idx_inertial+1 < size(obj.data_inertial,1)
                tni = obj.data_inertial( obj.idx_inertial+1, 1 );
            end
            if obj.idx_gps+1 < size(obj.data_gps,1)
                tng = obj.data_gps( obj.idx_gps+1, 1 );
            end
            
            if tni < tng
                % make inertial measurement
                obj.idx_inertial = obj.idx_inertial + 1;
                data = obj.data_inertial(obj.idx_inertial,:);
                m = Measurement( obj.MSGTYPE_INERTIAL, data(1) );
                m.setInertialData(data(2:4)', data(5:7)', data(8), data(9))
                return;
            else
                % make GPS measurement
                obj.idx_gps = obj.idx_gps + 1;
                data = obj.data_gps(obj.idx_gps,:);
                m = Measurement( obj.MSGTYPE_GPS, data(1) );
                m.setGPSData(data(2), data(3), data(4))
                return;
            end
            % otherwise we're out of data, return empty
            m = [];
        end
        
        % advance measurements to spot of first GPS data
        function advanceToFirstFix(obj)
            % first fix time
            tff = obj.data_gps(1,1);
            % advance inertial data
            ti = obj.data_inertial(obj.idx_inertial,1);
            while ti < tff
                obj.idx_inertial = obj.idx_inertial + 1;
                ti = obj.data_inertial(obj.idx_inertial,1);
            end
            % subtract one index so GPS is the first measurement
            obj.idx_inertial = obj.idx_inertial - 1;
        end
        
    end
    
end











































