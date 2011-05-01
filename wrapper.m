clear; clc


P_i = diag([1,1,1,1e-3,1e-3,1e-3]);
% X_hat with nominal values of wheel radii and wheel base length.
x_hat_i = [0, 0, 0, 1/pi, 1/pi, 0.5461];

R_imu = 1e-1;
R_gps = eye(2);
Q = diag([1e-3 1e-3 1 1e-6 1e-6 1e-6]);

model = LawnmowerModel(x_hat_i, P_i, Q, R_gps, R_imu);
model_uncorrected = LawnmowerModel(x_hat_i, P_i, Q, R_gps, R_imu);

f = 30;
t = 1:1/f:100;


u = encoders(:,2:3);

for ii = 1:length(t),
    if mod(ii,100),
        x_hat(ii,:) = model.TimeUpdate(u(ii,:),1/f);
        x_hat(ii,:) = model.MeasUpdateIMU(R_imu*randn,1/f);
    else
        [x_hat(ii,:),P_c] = model.TimeUpdate(u(ii,:),1/f);
        e = likelihood(x_hat(ii,1:2)',P_c(1:2,1:2),1);
        plot(e(1,:),e(2,:),'r')
    end
    [x_hat_u(ii,:),P_u] = model_uncorrected.TimeUpdate(u(ii,:),1/f);
  
end

plot(x_hat(:,1),x_hat(:,2),x_hat_u(:,1),x_hat_u(:,2),'g')
hold on;
e = likelihood(x_hat(ii,1:2)',P_c(1:2,1:2),1);
plot(e(1,:),e(2,:),'r')

% figure();
% plot(x_hat(:,3))