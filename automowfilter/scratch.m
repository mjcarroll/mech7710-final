addpath('../');
load_data;

%%
imu_data(:,1) = imu_data(:,1) - imu_data(1,1);
utm_data(:,1) = utm_data(:,1) - utm_data(1,1);

%% UTM vs IMU Data
figure();
subplot(3,1,1)
plot(imu_data(:,1),imu_data(:,4));

subplot(3,1,2)
plot(utm_data(:,1),utm_data(:,2));

subplot(3,1,3)
plot(utm_data(:,1),utm_data(:,3));

%% Plot states of x_hat to x_hat_u

iStart = 1;      iEnd = 2000;

figure(1);

for ii = 1:3
    subplot(3,1,ii)
    plot(time(iStart:iEnd),x_hat(iStart:iEnd,ii),...
        time_u(iStart:iEnd),x_hat_u(iStart:iEnd,ii),'g')
    title(strcat('$$\hat{X}_',...
        sprintf('%i',ii),'$$ with GPS vs $$\hat{X}_',...
        sprintf('%i',ii),'$$'),'interpreter','latex')
end

figure(2);
for ii = 1:3
    subplot(3,1,ii)
    plot(time(iStart:iEnd),x_hat(iStart:iEnd,ii+3),...
        time_u(iStart:iEnd),x_hat_u(iStart:iEnd,ii+3),'g')
    title(strcat('$$\hat{X}_',...
        sprintf('%i',ii+3),'$$ with GPS vs $$\hat{X}_',...
        sprintf('%i',ii+3),'$$'),'interpreter','latex')
end

%%