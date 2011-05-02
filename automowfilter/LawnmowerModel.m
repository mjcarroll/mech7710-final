classdef LawnmowerModel<handle
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties(Dependent = false, Constant = false)
        x_hat;
        P;
        Q;
        R_gps;
        R_imu;
        u_prev;
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
                obj.x_hat = ones(obj.nx,1);
                obj.P = eye(obj.nx);
                obj.Q = eye(obj.nx);
                obj.R_gps = eye(obj.ny_gps);
                obj.R_imu = eye(obj.ny_imu);
                obj.u_prev = zeros(obj.nu,1);
                obj.F = zeros(6);
            else
                if(size(x_hat_i,1) ~= obj.nx)
                    assert(size(x_hat_i,2)==obj.nx,...
                        'LawnmowerModel Constructor Error: Incorrect x_hat_i Size');
                   obj.x_hat = x_hat_i'; 
                else
                    obj.x_hat = x_hat_i;
                end
                assert(length(P_i) == obj.nx, ...
                    'LawnmowerModel Constructor Error: Incorrect P Size');
                obj.P = P_i;
                assert(length(Q) == obj.nx, ...
                    'LawnmowerModel Constructor Error: Incorrect Q Size');
                obj.Q = Q;
                assert(length(R_gps) == obj.ny_gps, ...
                    'LawnmowerModel Constructor Eror: Incorrect R_gps Size');
                obj.R_gps = R_gps;
                assert(length(R_imu) == obj.ny_imu, ...
                    'LawnmowerModel Constructor Eror: Incorrect R_imu Size');
                obj.R_imu = R_imu;
                obj.u_prev = zeros(obj.nu,1);
                obj.F = zeros(6);
            end
            return
        end
        
        function [x, P] = TimeUpdate(obj, u, dt)
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
            
%             obj.W = zeros(6,2);
%             W(1,1) = 1/2 * dt * x_hat(4) * cos(x_hat(3));
%             W(1,2) = 1/2 * dt * x_hat(5) * cos(x_hat(3));
%             W(2,1) = 1/2 * dt * x_hat(4) * sin(x_hat(3));
%             W(2,2) = 1/2 * dt * x_hat(5) * sin(x_hat(3));
%             W(3,1) = -dt * x_hat(4)/x_hat(6);
%             W(3,2) = dt * x_hat(5)/x_hat(6);
%             
%             obj.Q = W *  * W' + 
            
            v = obj.x_hat(5)/2 * u(2) + obj.x_hat(4)/2 * u(1);
            w = obj.x_hat(5)/obj.x_hat(6) * u(2) ...
                - obj.x_hat(4)/obj.x_hat(6) * u(1);
            
            obj.x_hat(1) = obj.x_hat(1) + ...
                dt * v * cos(obj.x_hat(3) + dt * w/2);
            obj.x_hat(2) = obj.x_hat(2) + ...
                dt * v * sin(obj.x_hat(3) + dt * w/2);
            obj.x_hat(3) = obj.x_hat(3) + dt * w;
            
            obj.P = obj.F * obj.P * obj.F' + obj.Q;
            
            obj.u_prev = u;
            x = obj.x_hat;
            P = obj.P;
        end
        
        function [x_hat, P, innovation] = MeasUpdateGPS(obj, y_gps, dt)
            obj.TimeUpdate(obj.u_prev,dt);
            C_gps = [1, 0, 0, 0, 0, 0; 
                     0, 1, 0, 0, 0, 0];
            innovation = y_gps - C_gps * obj.x_hat;
            S = C_gps * obj.P * C_gps' + obj.R_gps;
            K = obj.P * C_gps'/S;
            obj.x_hat = obj.x_hat + K * innovation;
            obj.P = (eye(obj.nx) - K * C_gps) * obj.P;
            
            x_hat = obj.x_hat;
            P = obj.P;
        end
        
        function [x_hat, P, innovation] = MeasUpdateIMU(obj, y_imu, dt)
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

