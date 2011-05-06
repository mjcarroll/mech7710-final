
Ts = 0.02;

time = 0:Ts:60;

wheel_base_length = 0.5461;
wheel_diameter = 0.158*2;
wheel_circum = wheel_diameter*pi;

cmd_vel = zeros(2, length(time));
cmd_vel(1,:) = 0.5*ones(1,length(time));
cmd_vel(2,:) = 0.01*ones(1,length(time)) + 0.1*sin(time/10);

% Initialize Data Structures
sim_u = zeros(2, length(time));
truth = zeros(3, length(time));
imu_data = zeros(length(time), 7);
encoder_data = zeros(length(time/2), 3);
utm_data = zeros(length(time/10),6);
utm_data(:, 4:6) = 0.1*ones(length(time), 3);

% Sensor Noise Characterists
imu_noise = 0.0012*randn(1,length(time));
imu_bias = 0.1;

encoder_noise = 0.2*randn(2, length(time/2));
encoder_bias = [0 0];

utm_noise = zeros(2, length(time/10));
utm_noise(1,:) = 0.04*randn(1,length(time/10));
utm_noise(2,:) = 0.04*randn(1,length(time/10));

utm_noise(:, 900:2000) = utm_noise(:, 900:2000)/0.04; % 1 meter
utm_data(900:2000, 4:6) = utm_data(900:2000, 4:6)*(30/0.1); % 3 meter

utm_bias = [0.0 0.0];

time_noise = 0.0*randn(length(time));

% Simulate Truth and Sensors
for ii=2:length(time)
    % Update Input
    v = cmd_vel(1,ii);
    w = cmd_vel(2,ii);
    v_l = ((v - (((v*wheel_base_length) / (v/w))/2)) / wheel_circum)*2*pi;
    v_r = ((v + (((v*wheel_base_length) / (v/w))/2)) / wheel_circum)*2*pi;
    sim_u(:,ii) = [v_l v_r];
   
    % Update the truth state
    truth(3,ii) = wrapToPi(truth(3, ii-1) + w*Ts);
    truth(1,ii) = truth(1, ii-1) + v*Ts*cos(truth(3,ii));
    truth(2,ii) = truth(2, ii-1) + v*Ts*sin(truth(3,ii));

    % Update the IMU everytime
    imu_data(ii,1) = time(ii) + time_noise(ii);
    imu_data(ii,4) = truth(3,ii) + imu_noise(ii) + imu_bias;
    imu_data(ii,5:7) = [0.0012 0.0012 0.0012];

    % Update Encoders at every other step
    if(mod(ii, 2) == 0)
       encoder_data(ii/2, 1) = time(ii) + time_noise(ii);
       encoder_data(ii/2, 2) = v_l+encoder_noise(1,ii/2)+encoder_bias(1);
       encoder_data(ii/2, 3) = v_r+encoder_noise(2,ii/2)+encoder_bias(2);
    end

    % Update GPS at every 10th step
    if(mod(ii, 10) == 0)
      utm_data(ii/10, 1) = time(ii) + time_noise(ii);
      utm_data(ii/10, 2) = truth(1,ii) + utm_noise(1,ii) + utm_bias(1);
      utm_data(ii/10, 3) = truth(2,ii) + utm_noise(2,ii) + utm_bias(2);
    end
end

% %%
% 
% figure(1);
% plot(truth(1,:), truth(2,:));
% hold on; axis equal;
% scatter(utm_data(:,2), utm_data(:,3));
% legend('Truth position', 'GPS UTM Position');
% ylabel('Northing');
% xlabel('Easting');
% 
% %%
% 
% figure(2);
% plot(time, truth(3,:));
% hold on;
% plot(time, imu_data(:,4), 'r--');
% legend('Heading Truth', 'Imu Simulated');
