clear
close all
clc

%% Coordinated Surveillance
N    = 200;                   % simulation duration
Npre = 500;                   % transient phase duration
Simulation_preprocessing;     % load data

% SEBS Initialization 
Np = NoC;             % all cameras as patrolling agents
s1 = round(0.5/Tc);   % step size
env.IvReset();
env.heatReset();
d = zeros(NoC, 2);    % stores the cameras displacement
detected = false;     % has a target been detected ?

%% ===================== Transient phase of the simulation ========================
% The transient phase in a simulation is often used to allow the system to reach a steady state before the actual data collection begins. 
% The transient phase here allows the cameras to spread out and cover the environment,
% and for the environment's idleness and heat to stabilize. 
% This ensures that the results of the simulation are not excessively influenced
% by the initial positioning of the cameras or the initial state of the environment.

% N.B env.v = its rows represent the coordinates of a vertex in env

disp('====================== Start of the Simulation =======================');
for t = 1:Npre
    Sa = zeros(1, length(env.V));             % Initialize vertex intentions
    for n = randperm(NoC)                     % iterating over cameras randomly
        % SEBS
        if mod(t, s1) == 0
            v(n) = C(n).SEBS(env, Np, Sa, L, M);       % Select next step
            d(n,:) = env.Vm(v(n),:) - C(n).X(1:2);     % displacement for each n-th camera

            % Share Intentions between cameras, with a chance of transmission errors 
            view = env.A(env.Vmap(v(n)), :);         % vertices that are visible from v(n)
            view(1,env.Vmap(v(n))) = 1;
            % Simulate transmission errors
            if randi([0,100])/100 > e_tx
                Sa(logical(view)) = Sa(logical(view)) + 1; % increments the intention of all vertices that are visible from v(n).
            end
        end
    end
    % MOVE cameras and updates the environment's idleness and heat, with a chance of transmission errors
    for n = 1:NoC
       C(n).V = v(n);                          % target vertex of the n-th camera to v(n)                  
       C(n).X(1:2) = C(n).X(1:2) + d(n,:)/s1;  % update the position of current camera
       % Simulate transmission errors
       if randi([0,100])/100 > e_tx
            env.IvUpdate(t*Tc, C(n).V)          % Update Idleness of vertex C(n).V
            env.deheat(C(n).V)                  % Deheat viewed vertices by camera n (decreases the heat of the vertex C(n).V in the environment, 
                                                % simulating the effect of the camera's presence)
       end
    end
end
% ============================== Transiente phase end =============================

%% Tracking Control Initialization
nL = 0;            % loss counter
s = round(T/Tc);
realZ = [];
% Initialization of the height controller's initial height h0 to the maximum
% height zMax of the first camera C(1), and the initial vertical velocity vh0 to 0
hctrl.h0 = C(1).zMax;
hctrl.vh0 = 0;

%   open Simulink model
open_system('control.slx');

figure()
for t = Npre+1:N+Npre
    env.plotBorders();
    hold on
    % env.plotGraphs();
    % hold on
    
    % TARGET TRAJECTORY
    if t > Npre+50
        x = (Ac*x' + Qc*randn(4,1))'; % New target Trajectory point
        % verify if new trajectory point is inside our environment
        in  = inpolygon(x(1), x(2), env.B(:,1), env.B(:,2));
        in1 = inpolygon(x(1), x(2), env.NFZ{1}(:,1), env.NFZ{1}(:,2));
        in2 = inpolygon(x(1), x(2), env.NFZ{2}(:,1), env.NFZ{2}(:,2));
        if in % inside the polygon defined by the points in B(:,1) and B(:,2) ?
            scatter(x(1), x(2), 30, [0.8500, 0.3250, 0.0980], 'filled')
        end
        % DETECTION
        firstStep = false;
        for n = 1:NoC
            % If a camera detects the target which is within the camera's FoV and not detected before
            if (abs(C(n).X(1)-x(1)) < C(n).FoV && abs(C(n).X(2)-x(2)) < C(n).FoV && ~detected && in && ~in1 && ~in2)
                C(n).Task = 1;                % tracking
                Np = Np-1;                    % decrease the number of patrolling agents since 1 is tracking
                env.addHeat(3, 1, 1, C(n).V); % adds heat to the env at the camera's vertex

                detected = true;
                firstStep = true;
                tDetect = t;
                % Initialize Filter (the first 2 lines of Algorithm 3)
                r = C(n).FoV*C(n).eFoV;                   
                R = r^2*eye(2);                           
                y = (H*x' + sqrt(R)*randn(2,1))';
                x_pred = [y,0 0];
                P_pred = blkdiag(R,eye(2));
                d(n,:) = x_pred(1:2) - C(n).X(1:2); % distance of the target from the camera
                break
            end
        end
    end
    
    % The agents are performing tasks and
    % making decisions based on sensor readings, predictions, and shared intentions

    Sa = zeros(1, length(env.V));             % Initialize vertex intentions / shared intentions of the agents
    for n = randperm(NoC)
        % SEBS
        if mod(t, s1) == 0 && C(n).Task == 0           % C(n) is patrolling
            v(n) = C(n).SEBS(env, Np, Sa, L, M);       % Select next step
            d(n,:) = env.Vm(v(n),:) - C(n).X(1:2);     % distance between the current and target/next camera position for the n-th agent
            % Share Intention
            view = env.A(env.Vmap(v(n)),:);
            view(1, env.Vmap(v(n))) = 1;
            Sa(logical(view)) = Sa(logical(view)) + 1;
        elseif mod(t, s) == 0 && C(n).Task == 1 && ~firstStep
            % agent attempts to take a sensor reading
            if randi([0,100])/100 > err && ...                 % sensor reading successfull
                abs(C(n).X(1)-x(1)) < C(n).FoV && ...
                abs(C(n).X(2)-x(2)) < C(n).FoV
                r = C(n).FoV*C(n).eFoV;                   
                R = r^2*eye(2);                           
                y = (H*x' + sqrt(R)*randn(2,1))';
                nL = 0;
            else                                                % sensor reading failed
                y = [0 0];  % loss of a measurement     
                nL = nL + 1;
            end

            % Predicton
            [x_pred, P_pred] = kalman(A, H, Q, R, y, x_pred, P_pred);
            d(n,:) = x_pred(1:2) - C(n).X(1:2);
            stdDev = sqrt(P_pred);
            stdDevMax = max((stdDev(1,1) + stdDev(2,2))/2 , T*(stdDev(3,3) + stdDev(4,4))/2);

            %% TRACKING LOSS
            % verify if the prediction is inside the allowed area and not inside any no-fly zones (NFZ2). 
            in  = inpolygon(x_pred(1), x_pred(2), env.B(:,1), env.B(:,2));
            in1 = inpolygon(x_pred(1), x_pred(2), env.NFZ{1}(:,1), env.NFZ{1}(:,2));
            in2 = inpolygon(x_pred(1), x_pred(2), env.NFZ{2}(:,1), env.NFZ{2}(:,2));
            % If the predicted position is outside the allowed area or inside a no-fly zone, or if the loss
            % counter is greater than 5, the agent stops its current task, 
            % adds heat to its current position in the environment, and selects a new vertex v(n) to move to
            if ~in || in1 || in2 || nL > 5
                C(n).Task = 0;
                Np = Np + 1;                                                          % return patrolling
                index = find(ismembertol(env.V, round(C(n).X(1:2)), 'ByRows', true)); % index of the vertex in the environment that is closest to the n-th agent's current position in the env's vertex list
                env.addHeat(3, 1, 1, index);                                          % add heat to the agent's current position in env
                index1 = logical(env.Vmap==index);
                if sum(index1) == 0                                                   % agent's current pos not in vertex map ==> the agent finds a new vertex v(n) to move to and calculates its displacement d(n,:) to the new vertex
                    index  = find(env.A(index,:));
                    v(n)   = find(env.Vmap==index(1));
                    d(n,:) = env.Vm(v(n),:) - C(n).X(1:2);
                else
                    v(n)   = find(index1);
                    d(n,:) = env.Vm(v(n),:) - C(n).X(1:2);
                end
                detected = false;
            end
        end
    end

    
    % MOVE
    for n = 1:NoC
        if C(n).Task == 0                           % patrolling
            C(n).V = v(n);                     
            C(n).X(1:2) = C(n).X(1:2) + d(n,:)/s1;
            if C(n).X(3) - C(n).zMax > 0.01         % z camera position a little bit above the max value
                z = C(n).zMax;                      % set it to the max value
                sim('control');
                hctrl.h0 = symout.data(end,2);
                hctrl.vh0 = symout.data(end,1);
                C(n).updateZ(hctrl.h0);
            else
                C(n).updateZ(C(n).zMax);
            end
        elseif C(n).Task == 1                       % tracking
            C(n).X(1:2) = C(n).X(1:2) + d(n,:)/s;
            % ZOOM
            if t-tDetect >= Ts*s
                [z,k] = C(n).optimalZoom(gamma, stdDevMax);
                zoom = [z;k];
                sim('control');
                hctrl.h0 = symout.data(end,2);
                hctrl.vh0 = symout.data(end,1);
                C(n).updateZ(hctrl.h0);
            end
        end
        C(n).plot;
        env.IvUpdate(t*Tc, C(n).V)            % Update Idleness
        env.deheat(C(n).V)                   % Deheat viewed vertices
    end
    
    hold off
    pause(Tc)
end
disp('====================== End of Simulation =======================');