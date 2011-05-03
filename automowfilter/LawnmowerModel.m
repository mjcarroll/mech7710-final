classdef LawnmowerModel<handle
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties(Dependent = false, Constant = false)
        x_hat;
        P;
        Q;
        R_gps;
        R_imu;
        prev_u;
        prev_time;
        F;
    end
    
    properties(Constant = true)
        nx = 6;
        ny_gps = 2;
        ny_imu = 1;
        nu = 2;
    end
    
    methods
        function obj = LawnmowerModel(x_hat_i, P_i, Q, R_gps, R_imu)
            %LAWNMOWERMODEL     Automow EKF
            % Instantiates the Lawnmower Extended Kalman Filter
            % Input Arguments:
            % x_hat_i       -  Initial State of the Kalman Filter
            % P_i           -  Initial Uncertainty of the Kalman Filter
            % Q             -  Process Disturbance Noise Covariance
            % R_gps         -  GPS Measurement Noise Covariance
            % R_imu         -  IMU Measurement Noise Covariance
            if(nargin == 0)
                % Default case, initialize everything to some values.
                obj.x_hat = [zeros(obj.nx/2,1); ones(obj.nx/2,1)];
                obj.P = eye(obj.nx);
                obj.Q = eye(obj.nx);
                obj.R_gps = eye(obj.ny_gps);
                obj.R_imu = eye(obj.ny_imu);
                obj.prev_u = zeros(obj.nu,1);
                obj.F = zeros(6);
            else
                % Case with input arguemnts
                % Make sure that x_hat_i is properly sized, transpose if
                % necessary.
                if(size(x_hat_i,1) ~= obj.nx)
                    assert(size(x_hat_i,2)==obj.nx,...
                        'LawnmowerModel Constructor Error: Incorrect x_hat_i Size');
                   obj.x_hat = x_hat_i'; 
                else
                    obj.x_hat = x_hat_i;
                end
                % Assert that P_i is the proper size (nx by nx)
                assert(length(P_i) == obj.nx, ...
                    'LawnmowerModel Constructor Error: Incorrect P Size');
                obj.P = P_i;
                % Assert that Q is the proper size (nx by nx)
                assert(length(Q) == obj.nx, ...
                    'LawnmowerModel Constructor Error: Incorrect Q Size');
                obj.Q = Q;
                % Assert that R is the proper size for the GPS and IMU
                assert(length(R_gps) == obj.ny_gps, ...
                    'LawnmowerModel Constructor Eror: Incorrect R_gps Size');
                obj.R_gps = R_gps;
                assert(length(R_imu) == obj.ny_imu, ...
                    'LawnmowerModel Constructor Eror: Incorrect R_imu Size');
                obj.R_imu = R_imu;
                obj.prev_u = zeros(obj.nu,1);
                obj.prev_time = 0;
                % Initialize the model to zero, will get set on first
                % update.
                obj.F = zeros(6);
            end
            return
        end
        
        function UpdateModel(obj,u,dt)
            % Here is where we construct the discrete F matrix from our
            % state equations.
            obj.F = eye(6);
            obj.F(1,3) = -1/2 * dt * ...
                (obj.x_hat(4) * u(1) + obj.x_hat(5)*u(2)) ...
                * sin(obj.x_hat(3));
            obj.F(1,4) = 1/2 * dt * u(1) * cos(obj.x_hat(3));
            obj.F(1,5) = 1/2 * dt * u(2) * cos(obj.x_hat(3));
            obj.F(2,3) = 1/2 * dt * ...
                (obj.x_hat(4) * u(1) + obj.x_hat(5)*u(2)) ...
                * cos(obj.x_hat(3));
            obj.F(2,4) = 1/2 * dt * u(1) * sin(obj.x_hat(3));
            obj.F(2,5) = 1/2 * dt * u(2) * sin(obj.x_hat(3));
            obj.F(3,4) = -dt * u(1)/obj.x_hat(6);
            obj.F(3,5) = dt * u(2)/obj.x_hat(6);
            obj.F(3,6) = dt * ...
                (obj.x_hat(4) * u(1) - obj.x_hat(5) * u(2)) / ...
                obj.x_hat(6)^2;
            
            % Store this input value, it may be useful later.  Especially
            % in the case where measurement updates come significantly
            % faster than time updates.
            obj.prev_u = u;
        end
        
        function [x, P] = TimeUpdate(obj, u, time)
            %TIMEUPDATE - Performs the time update on the system model.
            % u - Input at the time, 2x1, V_L and V_R
            % time - time in seconds of the update
            dt = time - obj.prev_time;
            obj.prev_time = time;
            
            obj.UpdateModel(u,dt);
            
            v = obj.x_hat(5)/2 * u(2) + obj.x_hat(4)/2 * u(1);
            w = obj.x_hat(5)/obj.x_hat(6) * u(2) ...
                - obj.x_hat(4)/obj.x_hat(6) * u(1);
            
            obj.x_hat(1) = obj.x_hat(1) + ...
                dt * v * cos(obj.x_hat(3) + dt * w/2);
            obj.x_hat(2) = obj.x_hat(2) + ...
                dt * v * sin(obj.x_hat(3) + dt * w/2);
            obj.x_hat(3) = obj.x_hat(3) + dt * w;
            
            obj.P = obj.F * obj.P * obj.F' + obj.Q;
            
            x = obj.x_hat;
            P = obj.P;
        end
        
        function [x_hat, P, innovation] = MeasUpdateGPS(obj, y_gps)
            C_gps = [1, 0, 0, 0, 0, 0; 
                     0, 1, 0, 0, 0, 0];
            innovation = y_gps' - C_gps * obj.x_hat;
            S = C_gps * obj.P * C_gps' + obj.R_gps;
            K = obj.P * C_gps'/S;
            obj.x_hat = obj.x_hat + K * innovation;
            obj.P = (eye(obj.nx) - K * C_gps) * obj.P;
            
            x_hat = obj.x_hat;
            P = obj.P;
        end
        
        function [x_hat, P, innovation] = MeasUpdateIMU(obj, y_imu)
            C_imu = [0, 0, 1, 0, 0, 0]; 
            innovation = y_imu - C_imu * obj.x_hat;
            S = C_imu * obj.P * C_imu' + obj.R_imu;
            K = obj.P * C_imu'/S;
            obj.x_hat = obj.x_hat + K * innovation;
            obj.P = (eye(obj.nx) - K * C_imu) * obj.P;
            
            x_hat = obj.x_hat;
            P = obj.P;
        end
    end
    
end

