classdef Drone < handle
    properties
        ID     % number of the drone
        Task   % Patrolling(0)/Tracking(1)
        Theta  % Field of View Angle
        FoV    % Field of View or Environment resolution (as in PDF)
        zmin   % Minimum Height
        zMax   % Maximum Height
        eFoV   % Detection Error (Percentage of FoV)
        V      % Vertex (in movement domain)
        X      % Position [x,y,z]
        noisyX % Noisy Position [x,y,z]
        GPSdev % standard deviation of GPS
        speed  % drones constant speed for moving between vertices
    end
    methods
        % CONSTRUCTOR
        function drone = Drone(env, theta, v, efov, ID)
            drone.Task   = 0;
            drone.speed  = 10; 
            drone.ID     = ID;
            drone.Theta  = theta;
            drone.V      = v;
            drone.FoV    = env.Res;
            drone.eFoV   = efov;
            drone.zMax   = drone.FoV/tan(theta*pi/180);
            drone.zmin   = drone.zMax/4;
            drone.X      = [env.Vm(drone.V, 1), env.Vm(drone.V, 2), drone.zMax];
            drone.GPSdev = 0.2;
            drone.noisyX = [0, 0, 0];
        end
        

        function noisyX = getNoisyX(drone)
            drone.X = drone.GPSdev*randn(1, length(drone.X)) + drone.X; % calibrated unbiased sensor
            noisyX  = drone.X;
        end


        % HEIGHT UPDATE
        function updateZ(drone, newZ)
            if (newZ >= drone.zmin && newZ <= drone.zMax)
                drone.X(3) = newZ;
                drone.FoV  = newZ*tan(drone.Theta*pi/180);
            end
        end
        
        % =======================  SEBS ============================
        % This function is used to determine the next vertex that the camera should move to,
        % based on a set of criteria.

        % L = parameter for probability control of lower gains M
        % M = gain saturation
        % Sa = penalty associated to # of cameras intended to view one of the vertices covered moving to vA
        % R = number of robots / drones / cameras
          
        function nextV = SEBS(drone, env, R, Sa, L, M)
            vA = find(env.Am(drone.V,:));     % find adjacent vertices
            Htot = 0;
            for k = vA
                Htot = Htot + sum(env.H(logical(env.A(env.Vmap(k),:)))); % total heat of all adjacent vertices
            end
            PmGS = []; % probability of moving to vA given the gain Ga and Sa events
            for k = vA
                % formula 15
                Ga = min(drone.speed/env.Res*sum(env.Iv(logical(env.A(env.Vmap(k),:)))), M);     % minimal average idleness of current vertex vk
                
                % Convert Heat in Prior
                Pm = sum(env.H(logical(env.A(env.Vmap(k),:))))/Htot; % formula (20) = probability of moving to vA       
                PGm = L * exp(log(1 / L) / M * Ga); % likelihood associated to Ga given that we move to vA (formula 16)
                view = env.A(env.Vmap(k,:)); % array of the neighboor vertices
                view(1, env.Vmap(k)) = 1;
                s = sum(Sa(logical(view))); % Sa (formula 18)
                PSm = 2^(R-(s+1))/(2^R-1); % formula 17
                PmGS = [PmGS, Pm * PGm * PSm];
            end
            % v_{n+1} = argmax{P(move(vA)|Ga, Sa}
            [~, idx] = max(PmGS);  % find the index of the maximum probability;
            nextV    = vA(idx);    % new vertex selected
        end
        % =============================================================
        
        % OPTIMAL ZOOM
        % function used to find the optimal zoom level z and a parameter k for a camera
        % based on a cost function. The cost function is a combination of the information lost
        % due to distance and an uncertainty estimation, weighted by a factor gamma.
        
        function [z, k] = optimalZoom(drone, gamma, stdDev)
            A = [-tan(drone.Theta*pi/180) stdDev; 0 0]; % [-tan(theta), stdDev]
            b = [0; 0];                                 % [0          ,     0 ]
            % pursuit of acceptable starting point
            f = zeros(1,2); % assumes x0 is the initial point
            x0 = linprog(f, A, b, [], [], [drone.zmin 2], [drone.zMax 3]);
            if isempty(x0) % no feasible point was found
                z =  drone.zMax;
                k = 2;
                return
            end
            % Information Lost (due to distance)
            I = @(z) -(exp(z-drone.zmin)-1); % formula 30 pre-multiplied by -1
            % Uncertainty Estimation
            beta = 0.6161;
            u = @(k) (1-exp(-k/beta)); % Confidence in the tracking given the coverage factor k
            % Cost Function
            f = @(z,k) -(I(z) + gamma*u(k)); % formula 37
            fun = @(x) f(x(1), x(2));
            % NL Optimal Problem
            % The interior-point algorithm is a type of algorithm used for nonlinear optimization 
            % that works by approximating the problem with a sequence of simpler problems, 
            % solving these simpler problems, and combining their solutions to get a solution to
            % the original problem.
            options = optimoptions('fmincon','Algorithm','interior-point');
            options.Display = 'notify';
            x = fmincon(fun, x0, A, b, [], [], [drone.zmin 2], [drone.zMax 3], [], options);
            % Solves for the optimal values
            z = x(1);
            k = x(2);
        end
          
        
        % PLOT
        % used to visualize the position and field of view of a camera cam in a 2D plot
        function plot(drone)
            if drone.Task == 0 % patrolling 
                scatter(drone.X(1), drone.X(2), 30, 'filled','v','k');
            else % tracking the target
                scatter(drone.X(1), drone.X(2), 30, 'filled','v','g');
            end
            rectangle('Position', [drone.X(1)-drone.FoV drone.X(2)-drone.FoV 2*drone.FoV 2*drone.FoV]) % square of the camera's drone FoV
            axis equal
        end
    end
end
