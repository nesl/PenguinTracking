classdef RigidBody < handle
    %RIGIDBODY < handle
    %6 DOF rigid body with derivatives and name
    %rb = RigidBody(name, xyz, theta)
    %xyz is 3D column
    %theta is 3D column
    
    properties
        % state
        xyz;
        dxyz;
        theta;
        dtheta;
        % identifier
        name;
        % covariance
        cov_xyz = [1; 1; 1]; % m/s
        cov_dxyz = [1; 1; 1]; % m/s^2
        cov_theta = [0.5; 0.5; 0.5]; % rad/s
        cov_dtheta = [0.5; 0.5; 0.5]; % rad/s^2
    end
    
    methods
        % CONSTRUCTOR
        function obj = RigidBody( name, xyz_i, theta_i )
            obj.name = name;
            obj.xyz = xyz_i;
            obj.dxyz = [0;0;0];
            obj.theta = theta_i;
            obj.dtheta = [0;0;0];
        end
        
        % get vectorized state
        function x = getStateVector(obj)
            x = [
                obj.xyz;
                obj.dxyz;
                obj.theta;
                obj.dtheta;
                ];
        end
        
        % get process covariance
        function P = getProcessCovariance(obj)
            P = diag([obj.cov_xyz; obj.cov_dxyz; obj.cov_theta; obj.cov_dtheta]);
        end
        
        % set state variables
        function setStateVector(obj, x)
            obj.xyz = x(1:3);
            obj.dxyz = x(4:6);
            obj.theta = x(7:9);
            obj.dtheta = x(10:12);
        end
        
        % get name
        function n = getName(obj)
            n = obj.name;
        end
        
        % get xyz
        function xyz = getPosition(obj)
            xyz = obj.xyz;
        end
        
        % get theta
        function theta = getOrientation(obj)
            theta = obj.theta;
        end
            
    end
    
end

