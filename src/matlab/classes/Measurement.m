classdef Measurement < handle
    %MEASUREMENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Constant)
        MSGTYPE_INERTIAL = 0;
        MSGTYPE_GPS = 1;
    end
    
    properties
        % measurement type
        type = 0;
        % measurement time
        timestamp = 0;
        % inertial data, if used
        acc_xyz = [0;0;0];
        mag_xyz = [0;0;0];
        baro = 0;
        temp = 0;
        % gps data, if used
        latitude = 0;
        longitude = 0;
        altitude = 0;
        % covariance terms
        cov_acc = [0.01; 0.01; 0.01]; % units?
        cov_mag = [0.01; 0.01; 0.01]; % units?
        cov_baro = 1; % units?
        cov_temp = 0.5; % deg. C
        cov_latitude = 0;
        cov_longitude = 0;
    end
    
    methods
        % constructor
        function obj = Measurement(type, time)
            % process raw measurement
            % format: time, dest, src, seq, ts0, ..., ts5, fppwr, cirp, fploss
            obj.type = type;
            obj.timestamp = time;
        end
        
        % set measurement data
        function setInertialData(obj, acc_xyz, mag_xyz, baro, temp)
            obj.acc_xyz = acc_xyz;
            obj.mag_xyz = mag_xyz;
            obj.baro = baro;
            obj.temp = temp;
        end
        
        % set GPS data
        function setGPSData(obj, latitude, longitude)
            obj.latitude = latitude;
            obj.longitude = longitude;
        end

        % get the vectorized measurements
        function z = vectorize(obj)
            z = [];
            if obj.type == obj.MSGTYPE_INERTIAL
                z = [obj.acc_xyz; obj.mag_xyz; obj.baro; obj.temp];
            end
            if obj.type == obj.MSGTYPE_GPS
                z = [obj.latitude; obj.longitude];
            end
        end
        
        % get the covariance matrix, R
        function R = getCovariance(obj)
            r = [];
            if obj.type == obj.MSGTYPE_INERTIAL
                r = [obj.cov_acc; obj.cov_mag; obj.cov_baro; obj.cov_temp];
            end
            if obj.type == obj.MSGTYPE_GPS
                r = [obj.cov_latitude; obj.cov_longitude];
            end
            R = diag(r);
        end
                
        % get measurement type
        function t = getType(obj)
            t = obj.type;
        end
        
        function setType(obj, type)
            obj.type = type;
        end
        
        % get measurement time
        function t = getTime(obj)
            t = obj.timestamp;
        end
       
    end
    
end

