addpath('../');
load_data;

%%
imu_data(:,1) = imu_data(:,1) - imu_data(1,1);
utm_data(:,1) = utm_data(:,1) - utm_data(1,1);

%%
figure();
subplot(3,1,1)
plot(imu_data(:,1),imu_data(:,4));

subplot(3,1,2)
plot(utm_data(:,1),utm_data(:,2));

subplot(3,1,3)
plot(utm_data(:,1),utm_data(:,3));