clear; clc
%%
load imu_data.csv
load encoders.csv

encoders(:,1) = (encoders(:,1) - encoders(1,1)) * 1e-9;
imu_data(:,1) = (imu_data(:,1) - imu_data(1,1)) * 1e-9;

imu_data = imu_data(:,1:7);

imu_data = imu_data(1:5:end,:);

[r,p,y] = quat2angle(imu_data(:,4:7));

%%

P_i = diag([1,1,1,1e-3,1e-3,1e-3]);
% X_hat with nominal values of wheel radii and wheel base length.
x_hat_i = [0, 0, 0, 0.159, 0.159, 0.5461];

R_imu = 1e-4;
R_gps = eye(2);
Q = diag([1 1 1 1e-6 1e-6 1e-6]);

model = LawnmowerModel(x_hat_i, P_i, Q, R_gps, R_imu);
model_uncorrected = LawnmowerModel(x_hat_i, P_i, Q, R_gps, R_imu);

f = 25;

u = encoders(:,1:2);

for ii = 1:length(u),
        fprintf('%f \r',ii/length(u) * 100);
        x_hat(ii,:) = model.TimeUpdate(u(ii,:),1/f);
        x_hat(ii,:) = model.MeasUpdateIMU(y(ii),1/f);
        x_hat_u(ii,:) = model_uncorrected.TimeUpdate(u(ii,:),1/f);
end

plot(x_hat(:,1),x_hat(:,2))
hold on;

% figure();
% plot(x_hat(:,3))