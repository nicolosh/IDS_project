clc;
clear all;
close all;

%% Scenario
dt = 1e-2;
duration = 500;
time = (0:dt:duration)';
N = length(time);

radius = 1;
omega = 2 * pi * tanh(2 * time) / duration;
velocity = radius * omega;
acceleration = gradient(velocity, dt);
angle = cumtrapz(dt, omega);
dronePath = radius * [cos(angle) sin(angle)];
attitude = pi / 2 + angle;

% Define GPS and IMU noise parameters
gps_noise_std = 0.01; % GPS noise standard deviation
imu_accel_noise_std = 0.001; % IMU acceleration noise standard deviation
imu_omega_noise_std = 0.001; % IMU angular velocity noise standard deviation
% generate random noise for the uwb measurements
uwb_noise_std = 0.1;


% Generate GPS and IMU noise
gps_noise = gps_noise_std * randn(N, 2);
imu_accel_noise = imu_accel_noise_std * randn(N, 1);
imu_omega_noise = imu_omega_noise_std * randn(N, 1);
uwb_noise_noise = uwb_noise_std * randn(N, 1);

% Generate GPS and IMU measurements
gps_measurements       = dronePath + gps_noise;
imu_accel_measurements = acceleration + imu_accel_noise;
imu_omega_measurements = omega + imu_omega_noise;

% Generate random number of UWB anchors
min_anchors = 3; % Minimum number of anchors
max_anchors = 6; % Maximum number of anchors
num_anchors = randi([min_anchors, max_anchors]); % Random number of anchors
anchors = rand(num_anchors, 2) * 10 - 5; % Generate random positions for anchors within a square region [-5, 5] x [-5, 5]
% Generate UWB measurements
uwb_measurements = pdist2(dronePath, anchors) + uwb_noise_noise;

figure; hold on; axis equal;
plot(dronePath(:,1), dronePath(:,2));
plot(anchors(:,1),anchors(:,2), 'x', 'MarkerSize',10,'LineWidth',3)
xlabel('x (m)')
ylabel('y (m)')

%% Distances
distance = pdist2(dronePath, anchors);
figure;
plot(time,distance);
xlabel('time (s)');
ylabel('distance (m)')

%% Dead reckoning problem

x_sym = str2sym({'x'; 'y'; 'theta'; 'v'; 'gps_x'; 'gps_y'; 'imu_accel'; 'imu_omega'});
u_sym = str2sym({'a'; 'omega'});
vars_sym = [x_sym; u_sym];

f_sym = str2sym({ 
    'x + v * dt * cos(theta)';...
    'y + v * dt * sin(theta)';...
    'theta + omega * dt';...
    'v + a*dt';...
    'gps_x';...
    'gps_y';...
    'imu_accel';...
    'imu_omega';
    });

%% Linearization
A_sym = jacobian(f_sym, x_sym);
G_sym = jacobian(f_sym, u_sym);

% code generation
f_func = matlabFunction(subs(f_sym, 'dt', dt), 'Vars', {u_sym x_sym});
A_func = matlabFunction(subs(A_sym, 'dt', dt), 'Vars', {u_sym x_sym});
G_func = matlabFunction(subs(G_sym, 'dt', dt), 'Vars', {u_sym x_sym});

% Measurement
uwb_h = cell(height(anchors), 1);
for i= 1:size(anchors, 1)
    x_i = anchors(i, 1);
    y_i = anchors(i, 2);

    uwb_h{i} = ['norm([x - (' num2str(x_i) '), y - (' num2str(y_i) ')] )'];
end

gps_h = {'gps_x'; 'gps_y'};
imu_h = {'imu_accel'; 'imu_omega'};
h_sym = str2sym(['theta'; uwb_h; gps_h; imu_h]);
H_sym = jacobian(h_sym, x_sym);

h_func = matlabFunction(h_sym, 'Vars', {x_sym});
H_func = matlabFunction(H_sym, 'Vars', {x_sym});

%% Sensors

% odometry
Q =  diag([0.01, 0.005]);
u = [acceleration, omega];

% measurement
R = blkdiag(0.1, 0.7 * eye(height(anchors)), gps_noise_std * eye(2), diag([imu_accel_noise_std^2, imu_omega_noise_std^2]));
z = [attitude, uwb_measurements, gps_measurements, imu_accel_measurements, imu_omega_measurements];

figure;
subplot(2,2,1); hold on;
title('acceleration')
plot(time,acceleration);

subplot(2,2,2); hold on;
title('angular velocity');
plot(time,omega);

subplot(2,2,3); hold on;
title('attitude')
plot(time, attitude)

subplot(2,2,4); hold on;
title('distance');
plot(time,distance)

%% Filter (EKF)
x = zeros(height(x_sym), N);
x(:,1) = [0 0 attitude(1), 0, 0, 0, 0, 0];

P = zeros(height(x_sym), height(x_sym), N);
P(:,:,1) = diag([0, 0, R(1,1), 0, 0, 0, 0, 0]);

for i = 1:N-1
    % Prediction
    x(:, i + 1) = f_func(u(i, :)', x(:,i));
    A = A_func(u(i, :)', x(:, i));
    G = G_func(u(i, :)', x(:, i));
    P(:, :, i+1) = A * P(:, :, i) * A' + G * Q * G';

    % Update
    if mod(i, 50) == 0
        H = H_func(x(:, i+1));
        S = H * P(:, :, i+1) * H' + R;
        W = P(:, :, i+1) * H' / S;
        I = z(i, :)' - h_func(x(:, i+1));
    
        x(:, i+1) = x(:, i+1) + W*I;
        P(:, :, i+1) = P(:,:,i+1) - W * H * P(:, :, i+1);
    end
end

figure;
subplot(2,2,1);hold on; axis equal;
title('dronePath')
plot(dronePath(:,1), dronePath(:,2));
plot(x(1,:), x(2,:))

subplot(2,2,2); hold on;
title('Attitude')
plot(time, attitude)
plot(time, x(3,:))

subplot(2,2,[3 4]); hold on;
title('velocity')
plot(time, velocity)
plot(time, x(4,:))

figure;hold on;
title('Covariance')

for i =1:8
    plot(time, squeeze(P(i,i,:)))
end
legend({'x', 'y', 'theta', 'v', 'gps_x', 'gps_y', 'imu_accel', 'imu_omega'})

error_pos_x = dronePath(:,1) - x(1,:)';
error_pos_y = dronePath(:,2) - x(2,:)';

figure;
histogram(error_pos_x, 50)

figure;
histogram(error_pos_y, 50)
