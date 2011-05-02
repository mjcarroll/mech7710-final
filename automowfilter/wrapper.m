clear; clc
%% Load in input data
addpath('../');
load_data;

%% Initialization of Filter Variables

% Initial State of x_hat
% x_hat = [Easting, Northing, Phi, Radius_L, Radius_R, Wheelbase]
% Loaded with nominal values of wheel radius and wheelbase length
x_hat_i = [0, 0, 0, 0.159, 0.159, 0.5461];

% We have a relatively high degree of confidence in our "constants"
P_i = diag([1 1 1 1e-3 1e-3 1e-3]);

% Nominal Values of R and Q, for a non-adaptive filter.
R_imu = 1e-4;
R_gps = eye(2);
Q = diag([1 1 1 1e-6 1e-6 1e-6]);

% Instantiate the model/filter
model = LawnmowerModel(x_hat_i, P_i, Q, R_gps, R_imu);
model_uncorrected = LawnmowerModel(x_hat_i, P_i, Q, R_gps, R_imu);

%%
for ii = 1:length(u),
        fprintf('%f \r',ii/length(u) * 100);
        x_hat(ii,:) = model.TimeUpdate(u(ii,:),1/f);
        x_hat(ii,:) = model.MeasUpdateIMU(y(ii),1/f);
        x_hat_u(ii,:) = model_uncorrected.TimeUpdate(u(ii,:),1/f);
end

