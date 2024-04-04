clear
close all
clc

% ==============================================================
% Setting up and running a simulation of a camera tracking system
% ==============================================================

rng(0)                                              % Seed for reproducibility

%% Environment
B = [0 0; 0 9; 7 9; 7 6; 3 6; 3 3; 7 3; 7 0];       % C Environment
%B = [0 0; 0 7; 4 7; 4 3; 6 3; 6 0];                % L Environment

res = 1;

env = Environment(B,{});    
env.cellularize(res);

%% Generate Camera
Ts = 5;             % settling time for P before starting zoom
theta = 40;         % angle (1m^2 from 2m)
gamma = 12;         % trade off parameter
eFoV = 0.05;        % percentage error of view
err = 0.05;         % measure loss percentage

cam = Camera(env, theta, 10, eFoV);

%% Starting Point of Real Trajcetory
pos = randi([min(min(B)),max(max(B))], 1, 2);
bar = [(min(B(:,1)) + max(B(:,1)))/2, (min(B(:,2)) + max(B(:,2)))/2];

% verify if starting point is on environment borders
[~, on] = inpolygon(pos(1), pos(2), B(:,1), B(:,2));
if ~on
    while ~on
        pos = randi([min(min(B)),max(max(B))],1,2);
        [~,on] = inpolygon(pos(1),pos(2),B(:,1),B(:,2));
    end
end
% initial velocity
v = bar-pos;

% checks if a small step in the direction of the velocity would keep the object within env
in = inpolygon(pos(1) + v(1)/100,pos(2) + v(2)/100, B(:,1), B(:,2));
if ~in
    v = -v;
end

%% Real Trajectory model
a = 1;                        % speed [m/s]
Tc = 0.1;                     % sampling period [s]
v = a*(v)/norm(v);            % speed vector [vx,vy]
x = [pos,v];                  % state vector
sigma_q = 15;                 % process noise variance

% process noise covariance matrix
%Qc = q*[0 0 0 0;0 0 0 0;0 0 1 0;0 0 0 1]; % random speed (q = 0.x)
% random acceleration (q = x)
Qc = sigma_q*[Tc^4/4, 0, Tc^3/2, 0; 0, Tc^4/4, 0, Tc^3/2; Tc^3/2, 0, Tc^2, 0; 0, Tc^3/2, 0, Tc^2];

% transition matrix  
Ac = [1, 0, Tc, 0; 0, 1, 0, Tc; 0, 0, 1, 0; 0, 0, 0, 1]; 

%% Filter trajectory model
T = 0.2;              % sampling period of Kalman filter so f = 5 [Hz]
sigma_q = 10;         % process noise variance

% process noise covariance matrix (random acceleration)
Q = sigma_q*[T^4/4, 0, T^3/2, 0; 0, T^4/4, 0, T^3/2; T^3/2, 0, T^2, 0; 0, T^3/2, 0, T^2];

% transition matrix  
A = [1, 0, T, 0; 0, 1, 0, T; 0, 0, 1, 0; 0, 0, 0, 1]; 

%% Filter Measurement model
H = [1 0 0 0; 0 1 0 0];     % measurement matrix
r = cam.FoV*cam.eFoV;       % measurement noise standard deviation
R = r^2*eye(2);             % measurement noise covariance matrix

%% Height Controller Sinthesys
hctrl.A = [0 1; 1 0]; % formula 39
hctrl.B = [1; 0];
hctrl.C = [1 0; 0 1];
hctrl.D = [0; 0];

hctrl.C1 = [0 1];
hctrl.D1 = 0;

hctrl.sys = ss(hctrl.A, hctrl.B, hctrl.C1, hctrl.D1);

% Position controller specifications
hctrl.ts = T;          %   desired settling time (at 5%)
hctrl.Mp = 0.01;       %   desired overshoot

% Specifications for loop TF 
hctrl.d    = log(1/hctrl.Mp) / sqrt(pi^2 + log(1/hctrl.Mp)^2);                    %   damping factor
hctrl.wgc  = 3/hctrl.d/hctrl.ts;                                                  %   gain crossover freq [rad/s]                                 
hctrl.phim = 180/pi * atan(2*hctrl.d/sqrt(sqrt(1+4*hctrl.d^4)-2*hctrl.d^2));      %   phase margin [deg]

% Position controller data
hctrl.alpha = 4;

% Filtered derivative tf (formula 41)
hctrl.wc   = 10*hctrl.wgc; 
hctrl.numD = [hctrl.wc, 0];
hctrl.denD = [1, hctrl.wc];

% Control design
[hctrl.kp, hctrl.ki, hctrl.kd] = get_PID(hctrl.sys, hctrl.wgc, hctrl.phim, hctrl.alpha);

% Anti-Windup Gain
hctrl.kw = 1/hctrl.ts;

%% One Step Ahead Prediction
s   = round(T/Tc); % this case is 2
out = false;

% Control Initialization
realZ = [];
z = cam.zMax;
hctrl.h0  = cam.zMax;  % initial camera height
hctrl.vh0 = 0;        % initial camera velocity

%   open Simulink models
open_system('control.slx');

% Filter Initialization 
% NOTE it must be set in the point where the first detection has been done
y(1,:) = (H*x(1,:)' + sqrt(R)*randn(2,1))';
x_pred(1,:) = [y(1,:),0 0];
x_pred(1+s,:) = [y(1,:),0 0];

P_pred(:,:,1) = blkdiag(R,eye(2));
P_pred(:,:,1+s) = blkdiag(R,eye(2));

stdDev = sqrt(P_pred(:,:,1));
stdDevMax(1) = max((stdDev(1,1) + stdDev(2,2))/2 , T*(stdDev(3,3) + stdDev(4,4))/2);
stdDevMax(1+s) = max((stdDev(1,1) + stdDev(2,2))/2 , T*(stdDev(3,3) + stdDev(4,4))/2);

zoom(:,1) = [cam.zMax; 3];
zoom(:,1+s) = [cam.zMax; 3];
realZ(1,2) = cam.zMax;
realZ(1+s,2) = cam.zMax;

for i = 2:s
    % New Trajectory point
    x = [x; (Ac*x(i-1,:)' + Qc*randn(4,1))'];
    % verify if new trajectory point is inside our environment
    in = inpolygon(x(i,1), x(i,2), B(:,1), B(:,2));
    if ~in
        out = true;
    end
end

count = s;
if ~out
    i = s+1;
else
    i=0;
end
while i ~= 0
    % New Trajectory point
    x = [x; (Ac*x(i-1,:)' + Qc*randn(4,1))'];
    
    if count >= s
        count = 1;
        % Detection
        if randi([0,100])/100 > err
            r = cam.FoV*cam.eFoV;                   
            R = r^2*eye(2);                           
            y(i,:) = (H*x(i,:)' + sqrt(R)*randn(2,1))';
        else
            y(i,:) = [0 0];  % loss of a measure
        end
        % Prediction
        [x_pred(i+s,:), P_pred(:,:,i+s)] = kalman(A, H, Q, R, y(i,:), x_pred(i,:), P_pred(:,:,i));
        stdDev = sqrt(P_pred(:,:,i+s));
        stdDevMax(i+s) = max((stdDev(1,1) + stdDev(2,2))/2 , T*(stdDev(3,3)+ stdDev(4,4))/2);
        % Zoom Control
        if i >= Ts*s
            [z,k] = cam.optimalZoom(gamma, stdDevMax(i+s));
            zoom(:,i+s) = [z;k];
            
            sim('control');
            realZ = [realZ; symout.data(2:end,:)];
            hctrl.h0 = symout.data(end,2);
            hctrl.vh0 = symout.data(end,1);
            cam.updateZ(hctrl.h0);
        else
            zoom(:,i+s) = [cam.zMax; 3];
            realZ(i+s,:) = [0, cam.zMax];
        end
    else
        count = count + 1;
    end
    
    % verify if new trajectory point is inside our environment
    in = inpolygon(x(i,1), x(i,2), B(:,1), B(:,2));
    if ~in
        i=0;
    else
        i = i+1;
    end
end

% Correct Indices
for t1 = 1:s:size(y,1)
    x_pred(t1+1:t1+s-1,:) = x_pred(t1+s,:).*ones(s-1,4);
    P_pred(:,:,t1+1:t1+s-1) = P_pred(:,:,t1+s).*ones(4,4,s-1);
    y(t1+1:t1+s-1,:) = y(t1,:).*ones(s-1,2);
end

% Evaluate Actual Coverage Factor
for t = 1:s:size(x,1)
    cam.updateZ(realZ(t,2));
    K(t) = cam.FoV/stdDevMax(t); % formula 33
end

%% Tracking performance index
pred = x_pred((Ts*s)+s:s:size(x,1),1:2);
real = x((Ts*s)+s:s:size(x,1),1:2);
track = sqrt((real(:,1)-pred(:,1)).^2 + (real(:,2)-pred(:,2)).^2);
track_err    = mean(track);
track_errMax = max(track);

%% Plot
figure()
for t = 1:size(x,1)
    env.plotBorders()
    hold on
    scatter(x(t,1),x(t,2),30,[0.8500, 0.3250, 0.0980],'filled')
    plot(x(1:t,1),x(1:t,2),'LineWidth',1.5,'Color',[0.8500, 0.3250, 0.0980])
    plot(y(1:t,1),y(1:t,2),'o','MarkerSize',3,'Color',[0, 0.4470, 0.7410])
    plot(x_pred(1:t,1),x_pred(1:t,2),'LineWidth',1.5,'Color',[0.9290, 0.6940, 0.1250])
    % Plot Camera
    cam.X(1) = x_pred(t,1);
    cam.X(2) = x_pred(t,2);
    cam.updateZ(zoom(1,t));
    cam.plot()
    hold off
    axis equal
    pause(Tc)
end

% Uncertainty
beta = 0.6161;
u = @(k) (1-exp(-k/beta));
% Information Lost (due to distance)
I = @(z) -(exp(z-cam.zmin)-1);

figure()
subplot(2,1,1)
plot(zoom(1,1:s:end))
xlabel('t[s]')
ylabel('z[m]')
title('Zoom level over time')
hold on
plot(realZ(1:s:end,2))
xlabel('t[s]')
ylabel('z[m]')
title('Real zoom level over time')
ylim([0,cam.zMax + 0.1])
subplot(2,1,2)
plot(zoom(2,1:s:end))
xlabel('t[s]')
ylabel('z[m]')
title('Zoom level over time')
hold on
plot(K(1:s:end))
xlabel('t[s]')
ylabel('-')
title('Actual Coverage Factor')

figure()
subplot(2,1,1)
plot(abs(I(realZ(1:s:end,2))))
xlabel('z[m]')
ylabel('-')
title('Information loss as function of real value of z')
subplot(2,1,2)
plot(u(zoom(2,1:s:end)))
xlabel('z[m]')
ylabel('-')
title('Uncertainty as function of the zoom level')
hold on
plot(u(K(1:s:end)))
title('Uncertainty as function of coverage Factor')


%% Zoom Analysis
figure()
for t = 1:s:size(x,1)
    % Plot Camera
    cam.X(1) = x_pred(t,1);
    cam.X(2) = x_pred(t,2);
    cam.updateZ(realZ(t,2));
    scatter(0,0,30,[0.9290, 0.6940, 0.1250],'filled')
    hold on
    rectangle('Position',[-cam.FoV -cam.FoV 2*cam.FoV 2*cam.FoV])
    scatter(x(t,1)-cam.X(1),x(t,2)-cam.X(2),30,[0.8500, 0.3250, 0.0980],'filled')
    rectangle('Position',[-stdDevMax(t) -stdDevMax(t) 2*stdDevMax(t) 2*stdDevMax(t)],'Curvature',1)
    rectangle('Position',[-3*stdDevMax(t) -3*stdDevMax(t) 2*3*stdDevMax(t) 2*3*stdDevMax(t)],'Curvature',1)
    rectangle('Position',[-2*stdDevMax(t) -2*stdDevMax(t) 2*2*stdDevMax(t) 2*2*stdDevMax(t)],'Curvature',1)
    hold off
    axis equal
    xlim([-env.Res-0.1 env.Res+0.1])    
    ylim([-env.Res-0.1 env.Res+0.1])
    pause(0.2)
end
