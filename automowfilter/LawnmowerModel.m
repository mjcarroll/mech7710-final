classdef LawnmowerModel<handle
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties(Dependent = false, Constant = false)
        x_hat;
        P;
        Q;
        R_gps;
        R_imu;
        prev_time;
        F;
        G;
        Radius_right = 0.158;
        Radius_left = 0.158;
        Wheelbase = 0.5461;
    end
    
    properties(Constant = true)
        nx = 4;
        ny_gps = 2;
        ny_imu = 2;
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
                obj.x_hat = [zeros(4,1)];
                obj.P = eye(obj.nx);
                obj.Q = eye(obj.nx);
                obj.R_gps = 5*eye(obj.ny_gps);
                obj.R_imu = eye(obj.ny_imu);
                obj.F = zeros(obj.nx);
                obj.G = zeros(obj.nx);
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
                obj.prev_time = 0;
                % Initialize the model to zero, will get set on first
                % update.
                obj.F = zeros(obj.nx);
                obj.G = zeros(obj.nx);
            end
            return
        end
        
        function UpdateModel(obj,u,dt)
            % Here is where we construct the discrete F matrix from our
            % state equations.
            
            v = obj.Radius_left * u(1)/2 + ...
                obj.Radius_right * u(2)/2;
            w = -obj.Radius_left * u(1)/obj.Wheelbase + ...
                obj.Radius_right * u(2)/obj.Wheelbase;
            
            obj.F = eye(obj.nx);
            obj.F(1,3) = dt * v * cos(dt * w);
            obj.F(1,4) = -dt * v * sin(dt * w);
            
            obj.F(2,3) = dt * v * cos(dt * w);
            obj.F(2,4) = dt * v * sin(dt * w);
            
            obj.F(3,3) = cos(dt*w);
            obj.F(3,4) = -sin(dt*w);
            
            obj.F(4,3) = cos(dt*w);
            obj.F(4,4) = sin(dt*w);
            
            phi_n_prime = obj.x_hat(3) * cos(dt * w) - ...
                obj.x_hat(4) * sin(dt * w);
            phi_e_prime = obj.x_hat(3) * cos(dt*w) + ...
                obj.x_hat(4) * sin(dt * w);
            
            obj.G = zeros(obj.nx);
            obj.G(1,1) = ...
                dt * obj.Radius_left * (1/2 * phi_n_prime - ...
                dt * v / obj.Wheelbase * ...
                (obj.x_hat(4) * cos(dt*w) + obj.x_hat(3) * sin(dt*w)));
            obj.G(1,2) = ...
                dt * obj.Radius_right * (1/2 * phi_n_prime - ...
                dt * v / obj.Wheelbase * ...
                (obj.x_hat(4) * cos(dt*w) + obj.x_hat(3) * sin(dt*w)));
            obj.G(2,1) = ...
                dt * obj.Radius_left * (1/2 * phi_e_prime + ...
                dt * v / obj.Wheelbase * ...
                (obj.x_hat(4) * cos(dt*w) - obj.x_hat(3) * sin(dt*w)));
            obj.G(2,2) = ...
                dt * obj.Radius_right * (1/2 * phi_e_prime + ...
                dt * v / obj.Wheelbase * ...
                (obj.x_hat(4) * cos(dt*w) - obj.x_hat(3) * sin(dt*w)));
            obj.G(3,1) = ...
                -dt * obj.Radius_left / obj.Wheelbase * ...
                (obj.x_hat(4) * cos(dt*w) + obj.x_hat(3) * sin(dt*w));
            obj.G(3,2) = ...
                -dt * obj.Radius_right / obj.Wheelbase * ...
                (obj.x_hat(4) * cos(dt*w) + obj.x_hat(3) * sin(dt*w));
            obj.G(4,1) = ...
                dt * obj.Radius_left / obj.Wheelbase * ...
                (obj.x_hat(4) * cos(dt*w) - obj.x_hat(3) * sin(dt*w));
            obj.G(4,2) = ...
                dt * obj.Radius_right / obj.Wheelbase * ...
                (obj.x_hat(4) * cos(dt*w) - obj.x_hat(3) * sin(dt*w));  
        end
        
        function [x, P] = TimeUpdate(obj, u, time)
            %TIMEUPDATE - Performs the time update on the system model.
            % u - Input at the time, 2x1, V_L and V_R
            % time - time in seconds of the update
            dt = time - obj.prev_time;
            obj.prev_time = time;
            
            obj.UpdateModel(u,dt);
            
            v = obj.Radius_left * u(1)/2 + ...
                obj.Radius_right * u(2)/2;
            w = -obj.Radius_left * u(1)/obj.Wheelbase + ...
                obj.Radius_right * u(2)/obj.Wheelbase;
            
            phi_n_prime = obj.x_hat(3) * cos(dt * w) - ...
                obj.x_hat(4) * sin(dt * w);
            phi_e_prime = obj.x_hat(3) * cos(dt*w) + ...
                obj.x_hat(4) * sin(dt * w);
            
            obj.x_hat(1) = obj.x_hat(1) + ...
                dt * v * phi_n_prime;
            obj.x_hat(2) = obj.x_hat(2) + ...
                dt * v * phi_e_prime;
            obj.x_hat(3) = phi_n_prime;
            obj.x_hat(4) = phi_e_prime;
            
            obj.P = obj.F * obj.P * obj.F' + obj.G * obj.Q * obj.G';
            
            x = obj.x_hat;
            P = obj.P;
        end
        
        function [x_hat, P, innovation] = MeasUpdateGPS(obj, y_gps, R_gps)
            if nargin == 2, 
                C_gps = [1, 0, 0, 0; 
                         0, 1, 0, 0];
                innovation = y_gps' - C_gps * obj.x_hat;
                S = C_gps * obj.P * C_gps' + obj.R_gps;
                K = obj.P * C_gps'/S;
                obj.x_hat = obj.x_hat + K * innovation;
                obj.P = (eye(obj.nx) - K * C_gps) * obj.P;

                x_hat = obj.x_hat;
                P = obj.P;
            else
                C_gps = [1, 0, 0, 0; 
                         0, 1, 0, 0];
                innovation = y_gps' - C_gps * obj.x_hat;
                S = C_gps * obj.P * C_gps' + R_gps;
                K = obj.P * C_gps'/S;
                obj.x_hat = obj.x_hat + K * innovation;
                obj.P = (eye(obj.nx) - K * C_gps) * obj.P;

                x_hat = obj.x_hat;
                P = obj.P;
            end
        end
        
        function [x_hat, P, innovation] = MeasUpdateIMU(obj, y_imu)
            C_imu = [0, 0, 1, 0;
                     0, 0, 0, 1]; 
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

