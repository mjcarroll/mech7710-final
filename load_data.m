addpath('~/Dropbox/Documentation/data/outside_broun_night_camera_2011-05-01-21-51-42/');
load utm_fix.csv
load imu.csv
load encoders.csv


utm = zeros(length(utm_fix),6);

utm(:,1) = utm_fix(:,1);
utm(:,2) = utm_fix(:,5) - utm_fix(1,5);
utm(:,3) = utm_fix(:,6) - utm_fix(1,6);
utm(:,4) = utm_fix(:,12);
utm(:,5) = utm_fix(:,16);
utm(:,6) = utm_fix(:,20);