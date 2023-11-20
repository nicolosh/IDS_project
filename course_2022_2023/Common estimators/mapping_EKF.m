clc;
clear;
close all;

%% scenario

dt = 1e-2;
duration = 10;
time = (0:dt:duration)';
N = length(time);

radius = 1;
omega = 2 * pi * tanh(2 * time) / duration;
velocity = radius * omega;
acceleration = gradient(velocity, dt);
angle = cumtrapz(dt, omega);
path = radius * [cos(angle) sin(angle)];
attitude = pi / 2 + angle;
anchors = [
    -1.5 0.0;
     2.5 0.0;
     0.0 1.5;
];

figure; hold on; axis equal;
plot(path(:, 1), path(:, 2));
plot(anchors(:, 1), anchors(:, 2), 'x', 'MarkerSize', 10, 'LineWidth', 3);
xlabel('x (m)');
ylabel('y (m)');

%% distances

target_distance = pdist2(path, anchors);
anchor_distance = pdist(anchors);

distance = [target_distance repmat(anchor_distance, N, 1)];

figure;
plot(time, distance);
xlabel('time (s)');
ylabel('distance (m)');

%% dead reckoning

target_x = {'x'; 'y'; 'theta'; 'v'};
anchor_x = cell(2 * height(anchors), 1);
for i = 1:height(anchors)
    anchor_x{2 * i - 1} = ['x_' num2str(i)];
    anchor_x{2 * i} = ['y_' num2str(i)];
end

x_sym = str2sym([target_x; anchor_x]);
u_sym = str2sym({'a'; 'omega'});
vars_sym = [x_sym; u_sym];

target_f = {
    'x + v * dt * cos(theta)';
    'y + v * dt * sin(theta)';
    'theta + omega * dt';
    'v + a * dt';
};

f_sym = str2sym([target_f; anchor_x]);

% linearization
A_sym = jacobian(f_sym, x_sym);
G_sym = jacobian(f_sym, u_sym);

% code generation
f_func = matlabFunction(subs(f_sym, 'dt', dt), 'Vars', {u_sym x_sym});
A_func = matlabFunction(subs(A_sym, 'dt', dt), 'Vars', {u_sym x_sym});
G_func = matlabFunction(subs(G_sym, 'dt', dt), 'Vars', {u_sym x_sym});

%% measurement

n_nodes = 1 + height(anchors);
n_edges = n_nodes * (n_nodes - 1) / 2;

uwb_h = cell(n_edges, 1);
for i = 1:height(anchors)
    x_i = anchors(i, 1);
    y_i = anchors(i, 2);
    uwb_h{i} = ['norm([x - x_' num2str(i) ', y - y_' num2str(i) '])'];
end
k = height(anchors) + 1;
for i = 1:height(anchors)
    for j = i+1:height(anchors)
        uwb_h{k} = [ ...
            'norm([' ...
            'x_' num2str(i) ' - x_' num2str(j) ', ' ...
            'y_' num2str(i) ' - y_' num2str(j) '])' ...
        ];
        k = k + 1;
    end
end

h_sym = str2sym(['x'; 'y'; 'theta'; uwb_h]);
H_sym = jacobian(h_sym, x_sym);

h_func = matlabFunction(h_sym, 'Vars', {x_sym});
H_func = matlabFunction(H_sym, 'Vars', {x_sym});

%% sensors

% odometry
Q = diag([0.01, 0.005]);
u = [acceleration omega] + mvnrnd(zeros(1, height(Q)), Q, N);

% measurement
R = blkdiag(0.01 * eye(2), 0.1, 0.001 * eye(n_edges));
z = [path attitude distance] + mvnrnd(zeros(1, height(R)), R, N);

figure;
subplot(2, 3, [1 4]); hold on; axis equal;
title('path');
plot(path(:, 1), path(:, 2));
plot(z(:, 1), z(:, 2), '.');
subplot(2, 3, 2); hold on;
title('acceleration');
plot(time, acceleration);
plot(time, u(:, 1), '.');
subplot(2, 3, 5); hold on;
title('angular velocity');
plot(time, omega);
plot(time, u(:, 2), '.');
subplot(2, 3, 3); hold on;
title('attitude');
plot(time, attitude);
plot(time, z(:, 3), '.');
subplot(2, 3, 6); hold on;
title('distance');
plot(time, distance);
plot(time, z(:, 4:end), '.');

%% extended kalman filter

x = zeros(height(x_sym), N);
x(:, 1) = [z(1, 1:3) 0 reshape(anchors', 1, []) + 0.1];

P = zeros(height(x_sym), height(x_sym), N);
P(:, :, 1) = blkdiag(R(1:3, 1:3), 0, 0.1 * eye(numel(anchors)));

for i = 1:N-1
    % predict
    x(:, i + 1) = f_func(u(i, :)', x(:, i));
    A = A_func(u(i, :)', x(:, i));
    G = G_func(u(i, :)', x(:, i));
    P(:, :, i + 1) = A * P(:, :, i) * A' + G * Q * G';

    % update
    if mod(i, 50) == 0
        H = H_func(x(:, i + 1));
        S = H * P(:, :, i + 1) * H' + R;
        W = P(:, :, i + 1) * H' / S;
        I = z(i, :)' - h_func(x(:, i + 1));
        x(:, i + 1) = x(:, i + 1) + W * I;
        P(:, :, i + 1) = P(:, :, i + 1) - W * H * P(:, :, i + 1);
    end
end

figure;
subplot(2, 2, 1); hold on; axis equal;
title('path');
plot(path(:, 1), path(:, 2));
plot(x(1, :), x(2, :));
subplot(2, 2, 2); hold on;
title('attitude');
plot(time, attitude);
plot(time, x(3, :));
subplot(2, 2, [3 4]); hold on;
title('velocity');
plot(time, velocity);
plot(time, x(4, :));

%% mapping error

mapping_error = zeros(N, height(anchors));
for i = 1:N
    anchor_est = reshape(x(5:end, i), 2, []);
    for j = 1:height(anchors)
        mapping_error(i, j) = norm(anchors(j, :) - anchor_est(:, j)', 2);
    end
end

figure; hold on;
plot(time, mapping_error);
title('mapping error');

%% covariance plot

figure; hold on;
title('covariace');
for i = 1:height(P)
    plot(time, squeeze(P(i, i, :)));
end