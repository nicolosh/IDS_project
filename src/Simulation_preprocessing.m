% Select Environment and cellularize it
res = 1;
env = selectEnvironment("Large", res);

figure(1)
env.plotBorders();

% Plot Graphs
figure(2)
env.plotGraphs();

%% Drones' Network
NoC = 8;                             % Number of Cameras
theta = 40;                           % angle (1m^2 from 2m)
gamma = 12;                           % trade-off parameter
eFoV = 0.05;                          % percentage error of view
err = 0.05;                           % measure loss percentage
e_tx = 0.05;                          % transmission error
v = randi(length(env.Vm), NoC, 1);    % Cameras initial position as matrix of random integers with NoC rows and 1 column between 1 and length(env.Vm)

Drones = Drone.empty(NoC, 0);         % empty array of cameras/drones objects 
for r = 1:NoC
    Drones(r) = Drone(env, theta, v(r), eFoV); % cameras initialization
end

% SEBS parameters
L = 0.1;
M = 9*length(env.V)/NoC;

%% Filter trajectory model
T = 0.2;              % sampling period of Kalman filter
sigma_q = 10;         % process noise standard deviation

% process noise covariance matrix (random acceleration)
Q = sigma_q*[T^4/4, 0, T^3/2, 0; 0, T^4/4, 0, T^3/2; T^3/2, 0, T^2, 0; 0, T^3/2, 0, T^2];

% transition matrix  
A = [1, 0, T, 0; 0, 1, 0, T; 0, 0, 1, 0; 0, 0, 0, 1]; 

% measurement model matrix
H = [1 0 0 0; 0 1 0 0]; 

% Simulation parameters
Tc = 0.1;               % sampling period (system)
Ts = 0.5;               % sampling period (communication (SEBS)) or % settling time for P before starting zoom
settlingTime   = 500;                
simulationTime = 200;             
samplingSEBS   = round(0.5/Tc);              % SEBS sampling = step-size
samplingKalman = round(T/Tc);                % Kalman sampling
d = zeros(NoC, 2);                           % stores the cameras displacement

% SEBS initialization
for t = 1:settlingTime
    Sa = zeros(1, length(env.V));             % Initialize vertex intentions
    for n = randperm(NoC)                     % iterating over cameras randomly
        % SEBS
        if mod(t, samplingSEBS) == 0
            v(n) = Drones(n).SEBS(env, Np, Sa, L, M);       % Select next step
            d(n,:) = env.Vm(v(n),:) - Drones(n).X(1:2);     % displacement for each n-th camera

            % Share Intentions between cameras, with a chance of transmission errors 
            view = env.A(env.Vmap(v(n)), :);         % vertices that are visible from v(n)
            view(1, env.Vmap(v(n))) = 1;
            % Simulate transmission errors
            if rand > e_tx
                Sa(logical(view)) = Sa(logical(view)) + 1; % increments the intention of all vertices that are visible from v(n).
            end
        end
    end
    % MOVE cameras and updates the environment's idleness and heat, with a chance of transmission errors
    for n = 1:NoC
        Drones(n).V = v(n);                          % target vertex of the n-th camera to v(n)                  
        Drones(n).X(1:2) = Drones(n).X(1:2) + d(n,:)/samplingSEBS;  % update the position of current camera
        % Simulate transmission errors
        if rand > e_tx
            env.IvUpdate(t*Tc, Drones(n).V)          % Update Idleness of vertex C(n).V
            env.deheat(Drones(n).V)                  % Deheat viewed vertices by drones's camera n (decreases the heat of the vertex C(n).V in the environment, 
                                                     % simulating the effect of the camera's presence)
        end
    end
end

%% Starting Point of Real targets Trajectories
pos = randi([min(min(env.B)), max(max(env.B))], 1, 2);
bar = [(min(env.B(:,1)) + max(env.B(:,1)))/2, (min(env.B(:,2)) + max(env.B(:,2)))/2]; % mid-point of env boundaries

% verify if starting point is on environment borders:
% if pos is not on the borders, it generates a new pos until it finds one that is on the borders.
[~, on] = inpolygon(pos(1), pos(2), env.B(:,1), env.B(:,2));
if ~on
    while ~on
        pos = randi([min(min(env.B)), max(max(env.B))], 1, 2);
        [~, on] = inpolygon(pos(1), pos(2), env.B(:,1), env.B(:,2));
    end
end
vT = bar - pos;
% verify if still within env by a small step in the velocity direction
in = inpolygon(pos(1) + vT(1)/100, pos(2) + vT(2)/100, env.B(:,1), env.B(:,2));
if ~in % not towards inwards into the env  
    vT = -vT;
end

%% Real Trajectory model
a = 1;                        % speed [m/s]
Tc = 0.1;                     % sampling period [s]
vT = a*(vT)/norm(vT);         % speed vector [vx,vy]
x = [pos, vT];                % state vector
sigma_q = 15;                 % process noise stdDev

% process noise covariance matrix
% Qc = sigma_q*[0 0 0 0;0 0 0 0;0 0 1 0;0 0 0 1]; % random speed (q = 0.x)
% random acceleration (q = x)
Qc = sigma_q*[Tc^4/4, 0, Tc^3/2, 0; 0, Tc^4/4, 0, Tc^3/2; Tc^3/2, 0, Tc^2, 0; 0, Tc^3/2, 0, Tc^2];

% transition matrix  
Ac = [1, 0, Tc, 0; 0, 1, 0, Tc; 0, 0, 1, 0; 0, 0, 0, 1]; 

%% Height Controller Sinthesys
hctrl.A = [0 1; 1 0];
hctrl.B = [1; 0];
hctrl.C = [1 0; 0 1];
hctrl.D = [0; 0];

hctrl.C1 = [0 1];
hctrl.D1 = 0;

hctrl.sys = ss(hctrl.A, hctrl.B, hctrl.C1, hctrl.D1);

% Position controller specifications
hctrl.ts = T;       %   desired settling time (at 5%)
hctrl.Mp = 0.01;    %   desired overshoot

% Get specifications for loop tf 
hctrl.d    = log(1/hctrl.Mp) / sqrt(pi^2 + log(1/hctrl.Mp)^2);                   %   damping factor
hctrl.wgc  = 3/hctrl.d/hctrl.ts;                                                 %   gain crossover freq [rad/s]                                 
hctrl.phim = 180/pi * atan(2*hctrl.d/sqrt(sqrt(1+4*hctrl.d^4)-2*hctrl.d^2));     %   phase margin [deg]

% Position controller data
hctrl.alpha = 4;

%   filtered derivative tf
hctrl.wc   = 10*hctrl.wgc; 
hctrl.numD = [hctrl.wc, 0];
hctrl.denD = [1, hctrl.wc];

% Control design
[hctrl.kp, hctrl.ki, hctrl.kd] = get_PID(hctrl.sys, hctrl.wgc, hctrl.phim, hctrl.alpha);

% Anti-Windup Gain
hctrl.kw = 1/hctrl.ts;
