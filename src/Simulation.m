clear
close all
clc

%% ===================== Transient phase of the simulation ========================
% The transient phase in a simulation is often used to allow the system to reach a steady state before the actual data collection begins. 
% The transient phase here allows the cameras to spread out and cover the environment,
% and for the environment's idleness and heat to stabilize. 
% This ensures that the results of the simulation are not excessively influenced
% by the initial positioning of the cameras or the initial state of the environment.

% N.B env.v = its rows represent the coordinates of a vertex in env

disp('\/\/\/\/\/\/\/\/\/\/\/ Start of the Simulation \/\/\/\/\/\/\/\/\/\/\/\/');
disp('====================== Start of Transiente phase =======================');
%% Coordinated Surveillance
Simulation_preprocessing;     % load data
env.IvReset();
env.heatReset();
detected = false;             % has a target been detected ?
disp('====================== End of the Transient phase =======================');


%% Tracking Control Initialization
nL = 0;            % loss counter
% realZ = [];
% Initialization of the height controller's initial height h0 to the maximum
% height zMax of the first drone(1), and the initial vertical velocity vh0 to 0
% hctrl.h0 = Drones(1).zMax;
% hctrl.vh0 = 0;

% Open Simulink model
% open_system('control.slx');

figure()
for t = (settlingTime + 1):(simulationTime + settlingTime)
    for i=1:NoC
        Drones(i).getNoisyX;
    end

    env.plotBorders();
    hold on
    
    % TARGET TRAJECTORY
    legendStrings = {};
    if t > settlingTime + 50
        x = (Ac*x' + Qc*randn(4,1))'; % New target Trajectory point
        % verify if new trajectory point is inside our environment
        in  = inpolygon(x(1), x(2), env.B(:,1), env.B(:,2));
        if in % inside the polygon defined by the points in B(:,1) and B(:,2) ?
            scatter(x(1), x(2), 30, [0.8500, 0.3250, 0.0980], 'filled');
        end
        in1 = inpolygon(x(1), x(2), env.NFZ{1}(:,1), env.NFZ{1}(:,2));
        in2 = inpolygon(x(1), x(2), env.NFZ{2}(:,1), env.NFZ{2}(:,2));
        % DETECTION
        firstStep = false;
        for n = 1:NoC
            % If a camera detects the target which is within the camera's FoV and not detected before
            if (abs(Drones(n).X(1) - x(1)) < Drones(n).FoV && abs(Drones(n).X(2) - x(2)) < Drones(n).FoV && ~detected && in && ~in1 && ~in2)
                Drones(n).Task = 1;                % tracking
                Np = Np-1;                         % decrease the number of patrolling agents since 1 is tracking
                env.addHeat(3, 1, 1, Drones(n).V); % adds heat to the env at the camera's vertex

                detected = true;
                firstStep = true;
                tDetect = t;
                % Drones(n).updateZ(Drones(n).zMax*0.7);

                % Initialize Filter (the first 2 lines of Algorithm 3)
                r = Drones(n).FoV * Drones(n).eFoV;                   
                R = r^2*eye(2);                           
                y = (H*x' + sqrt(R)*randn(2,1))';   % noisy camera reading
                x_pred = [y, 0, 0];
                P_pred = blkdiag(R, eye(2));
                d(n,:) = x_pred(1:2) - Drones(n).X(1:2); % distance of the target from the nth drone (innovation, is used to update the state estimate)
                % exit even if not all drones have been checked whether they detected the target or not
                break 
            end
        end
    end

    % The agents are performing tasks and
    % making decisions based on sensor readings, predictions, and shared intentions

    Sa = zeros(1, length(env.V));             % Initialize vertex intentions / shared intentions of the agents
    for n = randperm(NoC)          
        % SEBS
        if mod(t, samplingSEBS) == 0 && Drones(n).Task == 0           % Drone(n) is patrolling
            v(n) = Drones(n).SEBS(env, Np, Sa, L, M);                 % Select next step
            d(n,:) = env.Vm(v(n),:) - Drones(n).X(1:2);               % distance between the current and target/next camera position for the n-th agent
            % Share Intention
            view = env.A(env.Vmap(v(n)),:);
            view(1, env.Vmap(v(n))) = 1;
            Sa(logical(view)) = Sa(logical(view)) + 1;
        elseif mod(t, samplingKalman) == 0 && Drones(n).Task == 1 && ~firstStep
            % agent attempts to take a sensor reading
            if  rand > err && ...                 % sensor reading successfull
                abs(Drones(n).X(1)-x(1)) < Drones(n).FoV && ...
                abs(Drones(n).X(2)-x(2)) < Drones(n).FoV
                r = Drones(n).FoV*Drones(n).eFoV;                   
                R = r^2*eye(2);                           
                y = (H*x' + sqrt(R)*randn(2,1))';
                nL = 0;
            else                                                % sensor reading failed
                y = [0 0];                                      % loss of a measurement     
                nL = nL + 1;
            end

            % Predicton
            [x_pred, P_pred] = kalman(A, H, Q, R, y, x_pred, P_pred);
            d(n,:) = x_pred(1:2) - Drones(n).X(1:2);
            stdDev = sqrt(P_pred);
            stdDevMax = max((stdDev(1,1) + stdDev(2,2))/2 , T*(stdDev(3,3) + stdDev(4,4))/2); % formula 31

            %% TRACKING LOSS
            % verify if the prediction is inside the allowed area and not inside any no-fly zones (NFZ2). 
            in  = inpolygon(x_pred(1), x_pred(2), env.B(:,1), env.B(:,2));
            in1 = inpolygon(x_pred(1), x_pred(2), env.NFZ{1}(:,1), env.NFZ{1}(:,2));
            in2 = inpolygon(x_pred(1), x_pred(2), env.NFZ{2}(:,1), env.NFZ{2}(:,2));
            % If the predicted position is outside the allowed area or inside a no-fly zone, or if the loss
            % counter is greater than 5, the agent stops its current task, 
            % adds heat to its current position in the environment, and selects a new vertex v(n) to move to
            if ~in || in1 || in2 || nL > 5
                Drones(n).Task = 0;
                Np = Np + 1;                                                               % return patrolling
                index = find(ismembertol(env.V, round(Drones(n).X(1:2)), 'ByRows', true)); % index of the vertex in the environment that is closest to the n-th agent's current position in the env's vertex list
                env.addHeat(3, 1, 1, index);                                               % add heat to the agent's current position in env
                index1 = logical(env.Vmap==index);
                if sum(index1) == 0                                                        % agent's current pos not in vertex map ==> the agent finds a new vertex v(n) to move to and calculates its displacement d(n,:) to the new vertex
                    index  = find(env.A(index,:));
                    v(n)   = find(env.Vmap==index(1));
                else
                    v(n)   = find(index1);
                end
                d(n,:) = env.Vm(v(n),:) - Drones(n).X(1:2);
                detected = false;
            end
        end
    end

    
    % MOVE
    for n = 1:NoC
        if Drones(n).Task == 0                           % patrolling
            Drones(n).V = v(n);                     
            Drones(n).X(1:2) = Drones(n).X(1:2) + d(n,:)/samplingSEBS;
            if Drones(n).X(3) - Drones(n).zMax > 0.01         % z camera position a little bit above the max value
                %     z = Drones(n).zMax;                           % set it to the max value
                %     % sim('control');
                %     % hctrl.h0 = symout.data(end,2);
                %     % hctrl.vh0 = symout.data(end,1);
                Drones(n).updateZ(Drones(n).zMax*0.8);
            else
                Drones(n).updateZ(Drones(n).zMax);
            end
            legendStrings{end+1} = ['Drone ', num2str(Drones(n).ID), ' patrolling at height ', num2str(Drones(n).X(3))];
        elseif Drones(n).Task == 1                       % tracking
            
            Drones(n).X(1:2) = Drones(n).X(1:2) + d(n,:)/samplingKalman;
            % ZOOM
            if t-tDetect >= Ts*samplingKalman
                [z,k] = Drones(n).optimalZoom(gamma, stdDevMax);
                zoom = [z, k];
                % sim('control');
                % hctrl.h0 = symout.data(end,2);
                % hctrl.vh0 = symout.data(end,1);
                % Drones(n).updateZ(hctrl.h0);
                Drones(n).updateZ(zoom(1));
            end
            legendStrings{end+1} = ['Drone ', num2str(Drones(n).ID), ' tracking at height ', num2str(Drones(n).X(3))];
        end
        Drones(n).plot;
        env.IvUpdate(t*Tc, Drones(n).V)             % Update Idleness
        env.deheat(Drones(n).V)                     % Deheat viewed vertices
    end
    
    legend(legendStrings, 'Location', 'bestoutside');
    title('IDS simulator','interpreter','Latex','FontSize', 14);
    hold off
    pause(Tc)
end
disp('\/\/\/\/\/\/\/\/\/\/\/ End of Simulation \/\/\/\/\/\/\/\/\/\/\/\/\/');