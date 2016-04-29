classdef DataParser < handle
    %DATAPARSER parse tcp data logs from NTB nodes
    %   obj = DataParser( config_file, log_folder )
    
    
    properties
        data_inertial = {};
        data_gps = {};        
    end
    
    methods
        % Constructor
        function obj = DataParser( fp_inertial, fp_gps )
            
            % ===== read inertial data =====
            fid = fopen(fp_inertial);
            rawdata = textscan(fid, '%f %f %f %f %f %f %f %f %s %s %f', 'headerlines', 1);
            fclose(fid);
            % preallocate memory for inertial data
            obj.data_inertial = zeros(length(rawdata{1}), 9);
            % convert time to posix and append to data
            for i=1:length(rawdata{1})
                date = rawdata{9}{i};
                time = rawdata{10}{i};
                string = [date ' ' time];
                % days since 1-Jan-0000.
                dn = datenum(string, 'dd/mm/yyyy hh:MM:SS');
                % seconds since 1-Jan-0000.
                ds = dn*86400;
                obj.data_inertial(i,1) = ds;
            end
            % append accel
            obj.data_inertial(:,2:4) = [rawdata{1} rawdata{2} rawdata{3}];
            % append mag
            obj.data_inertial(:,5:7) = [rawdata{4} rawdata{5} rawdata{6}];
            % append temp
            obj.data_inertial(:,8) = rawdata{7};
            % append baro
            obj.data_inertial(:,9) = rawdata{8};
            
            % ===== read GPS data =====
            fid = fopen(fp_gps);
            rawdata = textscan(fid, '%s %s %f %f %f', 'headerlines', 1, 'delimiter', ',');
            fclose(fid);
            % preallocate memory for inertial data
            obj.data_gps = zeros(length(rawdata{1}), 4);
            % convert time to posix and append to data
            for i=1:length(rawdata{1})
                date = rawdata{1}{i};
                time = rawdata{2}{i};
                string = [date ' ' time];
                % days since 1-Jan-0000.
                dn = datenum(string, 'yyyy/mm/dd hh:MM:SS');
                % seconds since 1-Jan-0000.
                ds = dn*86400;
                obj.data_gps(i,1) = ds;
            end
            % append latitude
            obj.data_gps(:,2) = rawdata{3};
            % append longitude
            obj.data_gps(:,3) = rawdata{4};
            % append altitude
            obj.data_gps(:,4) = rawdata{5};
            
        end
        
    end
    
end











































