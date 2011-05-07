clear; clc
%% Load in input data
tic;
addpath('../');
load_data;
% load_simulation;
simulation = false;
toc

adaptive = false;
plot_ellipses = false;

if(~simulation)
    imu_data(:,4)= imu_data(:,4)-deg2rad(90);
end

%% Initialization of Filter Variables

% Initial State of x_hat
% x_hat = [Easting, Northing, Phi, Radius_L, Radius_R, Wheelbase, AHRS Bias]
% Loaded with nominal values of wheel radius and wheelbase length

x_hat_i = [0, 0, 0, 0.159, 0.159, 0.5461, 0, 0];

% We have a relatively high degree of confidence in our "constants"
P_i = diag([1 1 1 1e-3 1e-3 1e-3 1 1]);
% P_i = diag([0 0 0 0 0 0 0]);

% Nominal Values of R and Q, for a non-adaptive filter.
R_imu = 0.2;
R_gps = 0.1 * eye(2);
Q = diag([0.3 0.3 0 0 0 0 1e-6 1e-3]);


% Instantiate the model/filter
model = LawnmowerModel(x_hat_i, P_i, Q, R_gps, R_imu);
model_uncorrected = LawnmowerModel(x_hat_i, P_i, Q, R_gps, R_imu);
toc
%%

run = true;
time_index  = 1;
time_end = 75000;

iEncoder    = 1;
iIMU        = 1;
iGPS        = 1;

wc_length = length(encoder_data) + length(utm_data) + length(imu_data);
x_hat = zeros(wc_length,8);
time = zeros(wc_length,1);

time_index_u = 1;
wc_length_nogps = length(encoder_data) + length(imu_data);
x_hat_u = zeros(wc_length_nogps,8);
time_u = zeros(wc_length_nogps,1);

model.prev_time = imu_data(1,1);
model_uncorrected.prev_time = imu_data(1,1);

ii = 1;
figure(1), clf, axis equal;
while run == true,
    tEncoder = encoder_data(iEncoder,1);
    tGPS     = utm_data(iGPS,1);
    tIMU     = imu_data(iIMU,1);

    if tEncoder <= tIMU && tEncoder <= tGPS,
        % Add this to the time array.
        time(time_index) = tEncoder;
        % Do a time update
        x_hat(time_index,:) = model.TimeUpdate(encoder_data(iEncoder,2:3),tEncoder);
        time_index = time_index + 1;
        
        time_u(time_index_u) = tEncoder;
        x_hat_u(time_index_u,:) = model_uncorrected.TimeUpdate(encoder_data(iEncoder,2:3),tEncoder);
        time_index_u = time_index_u+1;
        
        iEncoder = iEncoder + 1;
        
    elseif tIMU < tEncoder && tIMU < tGPS,
        time(time_index) = tIMU;
        % Do a measurement update
        x_hat(time_index,:) = model.MeasUpdateIMU(imu_data(iIMU,4));
        time_index = time_index + 1;
        
        time_u(time_index_u) = tEncoder;
        x_hat_u(time_index_u,:) = model_uncorrected.MeasUpdateIMU(imu_data(iIMU,4));
        time_index_u = time_index_u+1;
        
        iIMU = iIMU + 10;
        
    elseif tGPS < tEncoder && tGPS <= tIMU
        time(time_index) = tGPS;
        if adaptive == true,
            [x_hat(time_index,:),P, inno] = model.MeasUpdateGPS(utm_data(iGPS,2:3),...
                diag([utm_data(iGPS,[4,5])].^2));
            innovations(:,iGPS) = inno;
            asdf(iGPS,1) = P(1,1);
            asdf(iGPS,2) = P(2,2);
            if plot_ellipses && mod(iGPS,20) == 0
                e = likelihood(x_hat(time_index,1:2)',P(1:2,1:2),1);
                plot(e(1,:),e(2,:),'g')
                clear e;
                ii = ii + 1;
            end
        else
            x_hat(time_index,:) = model.MeasUpdateGPS(utm_data(iGPS,2:3));
        end
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

time_end = time_index;

%%
figure(1);
plot(x_hat(1:time_end-1,1),x_hat(1:time_end-1,2), 'b');

xlabel('Easting'); ylabel('Northing');

hold on, axis equal, grid on;
plot(utm_data(1:iGPS,2),utm_data(1:iGPS,3),'r');
if(simulation)
    plot(truth(1,:), truth(2,:), 'k');
    legend('Estimated position', 'GPS position', 'Truth Position');
else
    legend('Estimated position', 'GPS position');
end

% %%
% qinc = 1;
% quiver(x_hat(1:qinc:time_end,1),x_hat(1:qinc:time_end,2),...
%     cos(x_hat(1:qinc:time_end,3)),sin(x_hat(1:qinc:time_end,3)),'g')
% quiver(utm_data(1:iGPS,2),utm_data(1:iGPS,3),...
%     cos(imu_data(1:20:iIMU+20,4)),sin(imu_data(1:20:iIMU+20,4)));
%
figure(2), clf;
plot(x_hat_u(1:time_index_u,1),x_hat_u(1:time_index_u,2), 'b+')
hold on, axis equal, grid on;
scatter(utm_data(1:iGPS,2),utm_data(1:iGPS,3),'r+')
legend('Estimated position', 'GPS position');

