classdef DataParser < handle
    %DATAPARSER parse tcp data logs from NTB nodes
    %   obj = DataParser( config_file, log_folder )
    
    
    properties
        MSGTYPE_INERTIAL = 0;
        MSGTYPE_GPS = 1;
        data_inertial = {};
        data_gps = {}; 
        messages = [];
        % UTM info
        utm_zone = '';
        % most recent measurements
        idx_inertial = 1;
        idx_gps = 1;
    end
    
    methods
        % Constructor
        function obj = DataParser( fp_inertial, fp_gps )
            % read inertial file
            obj.data_inertial = csvread(fp_inertial);
            
            % normalize magnetometer data
            norm_factor = sqrt(sum(obj.data_inertial(:,5:7).^2, 2));
            obj.data_inertial(:,5:7) = obj.data_inertial(:,5:7)./repmat(norm_factor, 1,3);
            
            % BP filter accel to get linear acceleration
            SR = 50;
            [bh,ah] = butter(2, 1/(SR/2), 'high');
            [bl,al] = butter(2, 10/(SR/2), 'low');
            obj.data_inertial(:,2) = filter(bh,ah,obj.data_inertial(:,2));
            obj.data_inertial(:,3) = filter(bh,ah,obj.data_inertial(:,3));
            obj.data_inertial(:,4) = filter(bh,ah,obj.data_inertial(:,4));
            obj.data_inertial(:,2) = filter(bl,al,obj.data_inertial(:,2));
            obj.data_inertial(:,3) = filter(bl,al,obj.data_inertial(:,3));
            obj.data_inertial(:,4) = filter(bl,al,obj.data_inertial(:,4));
            
            
            % read GPS file
            obj.data_gps = csvread(fp_gps);
            
            % append UTM info to GPS file for easier coordinates
            [x_utm, y_utm, zone_utm] = deg2utm( obj.data_gps(:,2), obj.data_gps(:,3) );
            obj.utm_zone = zone_utm(1);
            obj.data_gps = [obj.data_gps x_utm, y_utm];
        end
        
        % get the next measurement, ordered in time
        function m = getNextMeasurement(obj)
            % is the next measurement inertial or GPS?
            if obj.idx_inertial < size(obj.data_inertial,1)
                tni = obj.data_inertial( obj.idx_inertial, 1 );
            end
            if obj.idx_gps < size(obj.data_gps,1)
                tng = obj.data_gps( obj.idx_gps, 1 );
            end
            
            if tni < tng
                % make inertial measurement
                data = obj.data_inertial(obj.idx_inertial,:);
                m = Measurement( obj.MSGTYPE_INERTIAL, data(1) );
                m.setInertialData(data(2:4)', data(5:7)', data(8), data(9))
                obj.idx_inertial = obj.idx_inertial + 1;
                return;
            else
                % make GPS measurement
                data = obj.data_gps(obj.idx_gps,:);
                m = Measurement( obj.MSGTYPE_GPS, data(1) );
                x_utm = data(5);
                y_utm = data(6);
                alt = data(4);
                m.setGPSData(x_utm, y_utm, alt);
                obj.idx_gps = obj.idx_gps + 1;
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











































