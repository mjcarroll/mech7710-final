clear; clc
%% Load in input data
tic;
addpath('../');
load_data;
toc
%% Initialization of Filter Variables

% Initial State of x_hat
% x_hat = [Easting, Northing, Phi, Radius_L, Radius_R, Wheelbase]
% Loaded with nominal values of wheel radius and wheelbase length
x_hat_i = [0, 0, 0, 0.159, 0.159, 0.5461];

% We have a relatively high degree of confidence in our "constants"
P_i = diag([1 1 1 1e-3 1e-3 1e-3]);

% Nominal Values of R and Q, for a non-adaptive filter.
R_imu = 0.5;
R_gps = 0.05 * eye(2);
Q = diag([1 1 1 0 0 0]);

% Instantiate the model/filter
model = LawnmowerModel(x_hat_i, P_i, Q, R_gps, R_imu);
model_uncorrected = LawnmowerModel(x_hat_i, P_i, Q, R_gps, R_imu);
toc
%%

run = true;
time_index  = 1;
time_end    = 13000;

iEncoder    = 1;
iIMU        = 1;
iGPS        = 1;

wc_length = length(encoder_data) + length(utm_data) + length(imu_data);
x_hat = zeros(wc_length,6);
time = zeros(wc_length,1);


while run == true,
    tEncoder = encoder_data(iEncoder,1);
    tGPS     = utm_data(iGPS,1);
    tIMU     = imu_data(iIMU,1);

    if tEncoder < tIMU && tEncoder < tGPS,
        % Add this to the time array.
        time(time_index) = tEncoder;
        % Do a time update
        x_hat(time_index,:) = model.TimeUpdate(encoder_data(iEncoder,2:3),tEncoder);
        iEncoder = iEncoder + 1;
        time_index = time_index + 1;
    elseif tIMU < tEncoder && tIMU < tGPS,
        time(time_index) = tIMU;
        % Do a measurement update
        x_hat(time_index,:) = model.MeasUpdateIMU(imu_data(iIMU,2));
        iIMU = iIMU + 1;
        time_index = time_index + 1;
    elseif tGPS < tEncoder && tGPS < tIMU
        time(time_index) = tGPS;
        x_hat(time_index,:) = model.MeasUpdateGPS(utm_data(iGPS,2:3));
        iGPS = iGPS + 1;
        time_index = time_index + 1;
    end
    
    if iGPS == length(utm_data),
        run = false;
    end
    if iIMU == length(imu_data),
        run = false;
    end
    if iEncoder == length(encoder_data),
        run = false;
    end
    
    if time_index == time_end,
        run = false;
    end
    
end
toc

time = time - time(1);
%%
figure();
plot(x_hat(1:time_end,1),x_hat(1:time_end,2))
hold on;
plot(utm_data(1:iGPS,2),utm_data(1:iGPS,3),'g')