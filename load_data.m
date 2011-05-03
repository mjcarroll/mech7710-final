dataset = '~/Dropbox/Documentation/data/outside_broun_night_camera_nobase_2011-05-01-22-02-07/';
utm_fix = importdata(strcat(dataset,'utm_fix.csv'),';',1);
imu = importdata(strcat(dataset,'imu.csv'),';',1);
encoders = importdata(strcat(dataset,'encoders.csv'),';',1);

utm_fix = utm_fix.data;
encoders = encoders.data;
imu = imu.data;

%% Load UTM
% Load in UTM data into an array.
% Array Format:
% 1 - ROS Timestamp                 (s)
% 2 - Northing Position             (m)
% 3 - Easting Position              (m)
% 4 - Lattitude Covariance from GPS (m)
% 5 - Longitude Covariance from GPS (m)
% 6 - Altitude Covariance from GPS  (m)

utm_data = zeros(length(utm_fix),6);
utm_data(:,1) = utm_fix(:,1) * 1e-9;             % Convert to seconds
utm_data(:,2) = utm_fix(:,5) - utm_fix(1,5);     % Transform to origin
utm_data(:,3) = utm_fix(:,6) - utm_fix(1,6);     % Transform to origin
utm_data(:,4) = utm_fix(:,12);                   % Lattitude Covariance
utm_data(:,5) = utm_fix(:,16);                   % Longitude Covariance
utm_data(:,6) = utm_fix(:,20);                   % Altitude Covariance

%% Load Encoders
% Array Format:
% 1 - ROS Timestamp                 (s)
% 2 - Left Wheel Speed              (rad/s)
% 3 - Right Wheel Speed             (rad/s)

encoder_data = zeros(length(encoders),3);
encoder_data(:,1) = encoders(:,1) * 1e-9;
encoder_data(:,2:3) = encoders(:,2:3);

%% Load IMU
% Array Format:
% 1 - ROS Timestamp                 (s)
% 2 - Roll                          (rad)
% 3 - Pitch                         (rad)
% 4 - Yaw                           (rad)
% 5 - Roll Covariance               (rad)
% 5 - Pitch Covariance              (rad)
% 7 - Yaw Covariance                (rad)

imu_data = zeros(length(imu),7);
imu_data(:,1) = imu(:,1) * 1e-9;
[imu_data(:,2), imu_data(:,3), imu_data(:,4)] = quat2angle(imu(:,4:7));
imu_data(:,2) = imu_data(:,2);
imu_data(:,5:7) = imu(:,[8 12 16]);

%%
clear imu; clear utm_fix; clear encoders; clear dataset;