% Final Project - Imu Data Analysis
% William Woodall & Michael Carroll
% May 6th, 2011
% MECH 7710 Optimal Control and Estimation

clear; clc; close all; format loose; format short;

%% Import Static Data
% 

importfile('imu_data.csv');
clearvars textdata;

%% Calculate Statistics
% 

quat = zeros(length(data(:,1)), 4);

quat(:,1) = data(:,1);
quat(:,2) = data(:,2);
quat(:,3) = data(:,3);
quat(:,4) = data(:,4);

[yaw, pitch, roll] = quat2angle(quat);

%%
%

for ii=1:length(yaw)
    if yaw(ii) <= 0;
       yaw(ii) = 2*pi+yaw(ii);
    end
%     yaw(ii) = rad2deg(yaw(ii));
end

%%
%

yaw_mean = mean(yaw)
yaw_std = std(yaw)
yaw_range = range(yaw)

figure(1);
plot(yaw);
hold on;
line([1 length(yaw)], [yaw_mean yaw_mean], 'Color', 'k', 'LineStyle', '--');
line([1 length(yaw)], [yaw_mean+yaw_std yaw_mean+yaw_std], 'Color', 'r', 'LineStyle', '--');
line([1 length(yaw)], [yaw_mean-yaw_std yaw_mean-yaw_std], 'Color', 'r', 'LineStyle', '--');
yaw_std = yaw_std*2;
line([1 length(yaw)], [yaw_mean+yaw_std yaw_mean+yaw_std], 'Color', 'b', 'LineStyle', '--');
line([1 length(yaw)], [yaw_mean-yaw_std yaw_mean-yaw_std], 'Color', 'b', 'LineStyle', '--');
yaw_std = (yaw_std/2)*3;
line([1 length(yaw)], [yaw_mean+yaw_std yaw_mean+yaw_std], 'Color', 'g', 'LineStyle', '--');
line([1 length(yaw)], [yaw_mean-yaw_std yaw_mean-yaw_std], 'Color', 'g', 'LineStyle', '--');
yaw_std = yaw_std/3;