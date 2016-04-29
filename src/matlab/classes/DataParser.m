classdef DataParser < handle
    %DATAPARSER parse tcp data logs from NTB nodes
    %   obj = DataParser( config_file, log_folder )
    
    
    properties
        raw_inertial = {};
        raw_gps = {};        
    end
    
    methods
        % Constructor
        function obj = DataParser( fp_inertial, fp_gps )
            
            % read inertial data
            fid = fopen(fp_inertial);
            line = textscan(fid, '%f %f %f %f %f %f %f %f %f %f %s %s %f', 'Delimiter', ',', 'headerlines', 1);
            while ~isempty(line)
                % process posix time
                
                
                
            end
            obj.raw_inertial = textscan(fid, '%f %f %f %f %f %f %f %f %f %f %s %s %f', 'Delimiter', ',', 'headerlines', 1);
            fclose(fid);
            
        end
        
    end
    
end











































