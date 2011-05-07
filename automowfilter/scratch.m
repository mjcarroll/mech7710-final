addpath('../');
load_data;

%%
imu_data(:,1) = imu_data(:,1) - imu_data(1,1);
utm_data(:,1) = utm_data(:,1) - utm_data(1,1);


%%
load_data;
rollovers = 0;

for ii = 1:length(imu_data)-1
    if sign(imu_data(ii,4)) == -1
        imu_data(ii,4) = imu_data(ii,4) + 2* pi;
    end
end


plot(imu_data(:,4))

%% UTM vs IMU Data
figure();
subplot(3,1,1)
plot(imu_data(:,1),imu_data(:,4));
title('AHRS Phi');

subplot(3,1,2)
plot(utm_data(:,1),utm_data(:,2));
title('Easting');

subplot(3,1,3)
plot(utm_data(:,1),utm_data(:,3));
title('Northing');

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

%% IMU vs UTM calculated heading
figure(3);
subplot(2,1,1)
plot(imu_data(:,1),imu_data(:,4));

% [b, a] = butter(30,0.5);
% utm_new(:,1) = filter(b,a,utm_data(:,2));
% utm_new(:,2) = filter(b,a,utm_data(:,3));


utm_heading(1) = 0;
for ii=2:length(utm_data(:,1))
    utm_heading(ii) = atan((utm_data(ii,3)-utm_data(ii-1,3))/(utm_data(ii,2)-utm_data(ii-1,2)));
%     utm_heading(ii) = atan((utm_new(ii,2)-utm_new(ii-1,2))/(utm_new(ii,1)-utm_new(ii-1,1)));
end

%utm_heading(length(utm_data(:,1))+1) = utm_heading(length(utm_data(:,1)));

subplot(2,1,2)
plot(utm_data(:,1),utm_heading(:));


%%
figure(4),clf;
subplot(3,1,1)
hold on
scatter(imu_data(:,1),truth(1,:))
scatter(time(1:time_end),x_hat(1:time_end,1),'g+')
subplot(3,1,2)
hold on
scatter(imu_data(:,1),truth(2,:))
scatter(time(1:time_end),x_hat(1:time_end,2),'g+')
subplot(3,1,3)
hold on
scatter(imu_data(:,1),truth(3,:))
scatter(time(1:time_end),x_hat(1:time_end,3),'g+')


%%

figure(5), clf;
subplot(1,2,1);
plot(x_hat(1:time_end-1,1),x_hat(1:time_end-1,2), 'b')

xlabel('Easting'); ylabel('Northing');

hold on, axis equal, grid on;
plot(utm_data(1:iGPS,2),utm_data(1:iGPS,3),'r')
if(simulation)
    plot(truth(1,:), truth(2,:), 'k');
    legend('Estimated position', 'GPS position', 'Truth Position');
else
    legend('Estimated position', 'GPS position');
end
subplot(1,2,2);
plot(x_hat(1:time_end-1,1),x_hat(1:time_end-1,2), 'b')

xlabel('Easting'); ylabel('Northing');

hold on, axis equal, grid on;
plot(utm_data(1:iGPS,2),utm_data(1:iGPS,3),'r')
if(simulation)
    plot(truth(1,:), truth(2,:), 'k');
    legend('Estimated position', 'GPS position', 'Truth Position');
else
    legend('Estimated position', 'GPS position');
end

%%

figure(6);
plot(x_hat(1:time_end, 7));
hold on;
plot([1 time_end], [mean(x_hat(1:time_end,7)) mean(x_hat(1:time_end,7))], 'k--');
plot([1 time_end], [(-3.616667/180)*pi (-3.616667/180)*pi], 'r--');
ylabel('IMU Bias State (radians)');
xlabel('Time');
legend('IMU Bias Estimate', 'Mean Bias', 'Magnetic Declination for Auburn');


