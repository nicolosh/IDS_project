% ==============================================================
% This MATLAB script is running a simulation of a 
% camera network using the SEBS algorithm
% ==============================================================
load('Data/C.mat')      % load desired environment
env = C;                % rename selected environment
R = 2;                  % Number of Cameras
N = 130;                % Number of Iterations

% Camera Network
theta = 40;                         % AoV
e_tx = 0.05;                        % Transmission Error
v = randi(length(env.Vm), R, 1);    % Cameras initial position (SELECTED from the vertices in env)
C = Camera.empty(R, 0);
for r = 1:R
    C(r) = Camera(env, theta, v(r), 0.05);
end

% SEBS parameters
L = 0.1;
M = 9*length(env.V)/R;

%% SEBS Simulation
X = zeros(R, N);    % matrix to store the cameras paths
env.IvReset();
env.heatReset();
Hcheck = true;

% Each iteration selects the next step for each camera, simulates transmission errors, updates the
% env idleness and env heat, & checks if a specific zone has been deheated
for t = 1:N
    if t == 110
        % env.addHeat(2,1,1,10);                  % Simulate event detection
    end
    X(:, t) = v;                                  % Add the vertexes to the paths
    Sa = zeros(1, length(env.V));                 % Initialize vertex intentions
    for r = randperm(R)                           % looping over a random permutation of integer from 1 to R = 2 (i.e): it's used to randomize the order in which the cameras are processed in each iteration of the SEBS sim 
        v(r) = C(r).SEBS(env, R, Sa, L, M);        % Select next step
        % Share Intention
        view = env.A(env.Vmap(v(r)),:);            % current view of the rth camera (mapping pos of r-th camera to an env vertex)
        view(1, env.Vmap(v(r))) = 1;               % current pos of r-th camera to 1
        
        % Simulate transmission errors
        if randi([0,100])/100 > e_tx
            Sa(logical(view)) = Sa(logical(view)) + 1; % incrementing the elements of Sa that correspond to the current view of the camera. 
                                                       % Sa represents the "intention" of each vertex in the environment, so we are sharing the camera's intention with the vertices it is currently viewing
        end
    end
    for r = 1:R
        C(r).V = v(r);                        % Move r-th to v(r)
        % Simulate transmission errors
        if randi([0,100])/100 > e_tx
            env.IvUpdate(t, C(r).V)           % Update Idleness of env at the current position of the rth camera at time t
            env.deheat(C(r).V)                % Deheat viewed vertices by r-th camera
        end
    end
    if t >= 110 && env.H(10)==1 && Hcheck
        Hcheck = false;
        fprintf('Zone Deheated in %f steps. \n', t-110); % number of steps taken to deheat the zone since step 110
    end
end

%% Plot Results
% Plots the environment, the No-Fly Zones, 
% the positions and paths of the cameras, and the viewed vertices for each iteration from 103 to N

Xp = env.Vmap(X(:, 103:end)); % maps the paths of the cameras to the vertices in env
figure()
for i = 1:size(Xp,2) 
    plot(polyshape(env.B(:,1),env.B(:,2)),'FaceAlpha',0.2);
    hold on
    for n=1:length(env.NFZ)
        plot(polyshape(env.NFZ{n}(:,1),env.NFZ{n}(:,2)),'FaceAlpha',0.2);
    end
    for r = 1:R
        scatter(env.V(Xp(r,i),1), env.V(Xp(r,i),2),'filled');     % plot position
        plot(env.V(Xp(r,1:i),1), env.V(Xp(r,1:i),2));             % plot path
        for k=1:i
            scatter(env.V(logical(env.A(Xp(r,k),:)),1), env.V(logical(env.A(Xp(r,k),:)),2), 'b');   % highlight viewed vertices
        end
    end
    hold off
    axis equal
    pause(0.5);
end
close  % close figure after completed sim

%% Coverage Analysis (K1 tests of K2 coverages)
K1 = 1000;
K2 = 100;

% arrays to store the total coverage period, total idleness, and total deheated period for each test
Tvtot = zeros(1, K1);
Ivtot = zeros(1, K1); 
THcheck = zeros(1, K1);

for N = 1:K1
    % Reset Camera Positions
    v = randi(length(env.Vm), R, 1);     % Cameras initial position
    C = Camera.empty(R,0);
    for r = 1:R
        C(r) = Camera(env, theta, v(r), 0.05);
    end
    X = [v]; % cameras Paths
    env.IvReset();
    env.heatReset();
    % tHcheck is a random time step within the range from 1 to approximately 10 times the average number of vertices per camera.
    % This might be used to simulate a random event occurring at a certain time in the simulation
    Hcheck = true;
    tHcheck = randi([1, round(10*length(env.Vm)/R)]);
    n = 1;
    t = 0;
    Tv = zeros(1,K2);
    Iv = zeros(1,K2);
    % viewed is used to track which vertices have been viewed by the cameras. Each element in the 
    % array corresponds to a vertex in the environment, and is set to 1 when the corresponding vertex
    % has been viewed
    viewed = zeros(1, size(env.V, 1));
    
    % For each coverage, while loop simulates event detection, selects the next step for each camera, moves the cameras,
    % updates the idleness and heat of the environment, and checks if a specific zone has been deheated.
    while n <= K2
        if t == tHcheck
            vHcheck = randi(length(env.V));
            env.addHeat(2, 1, 1, vHcheck);             % Simulate event detection
        end
        Sa = zeros(1,length(env.V));                   % Initialize vertex intentions
        for r = randperm(R)
            v(r) = C(r).SEBS(env, R, Sa, L, M);        % Select next step
            % Share Intention
            view = env.A(env.Vmap(v(r)),:);
            view(1,env.Vmap(v(r))) = 1;
            Sa(logical(view)) = Sa(logical(view)) + 1; 
        end
        for r=1:R
            C(r).V = v(r);                             % Move
            if t > 100
                viewed = sign(viewed + env.A(env.Vmap(v(r)),:)); % adding adjacency matrix row corresponding to the current vertex of the camera
            end
            env.IvUpdate(t, C(r).V)                     % Update Idleness of vertex where the r-th is located
            env.deheat(C(r).V);                         % Deheat viewed vertices
        end
        X = [X v];                                      % Add the current camera vertexes to the paths
        if t >= tHcheck && env.H(vHcheck)==1 && Hcheck
            Hcheck = false;
            THcheck(N) = t-tHcheck;                     % records the time it took for the event to occur
        end
        t = t + 1;                                      % time step update
        Tv(n) = Tv(n) + 1;                              % updating coverage period of n-th coverage
        Iv(n) = Iv(n) + mean(env.Iv);                   % updating idleness of n-th coverage
        if sum(viewed) == length(viewed)                % if all vertices have been viewed
            viewed = zeros(1,size(env.V,1));            % resets the viewed array 
            n = n + 1;
        end
    end
    % the mean coverage period and mean idleness for the current test
    Tvtot(N) = mean(Tv(2:end))-1;
    Ivtot(N) = mean(Iv(2:end)./Tv(2:end));
end

fprintf('Mean Coverage Period: %2.3f \n', mean(Tvtot));
fprintf('Mean Idleness: %2.3f \n', mean(Ivtot));
fprintf('Mean Zone Deheated Period: %2.3f steps. \n', mean(THcheck));

%% Percentage Coverage Analysis (K1 tests at K2 steps)
K1 = 1000;
K2 = 12;

ViewTot = zeros(1, K1); 
for N = 1:K1
    % Reset Camera Positions
    v = randi(length(env.Vm), R, 1); % Cameras initial position
    C = Camera.empty(R,0);
    for r = 1:R
        C(r) = Camera(env, theta, v(r), 0.05);
    end
    X = []; % Paths
    env.IvReset();
    env.heatReset();
    % This is often done when K is used as a stopping condition for a loop,
    % and you want the loop to run indefinitely until some other condition is met
    K = inf; % infinite positive
    n = 1;
    t = 0;
    viewed = zeros(1, size(env.V,1)); % keeps track of vertices been viewed by the cameras
    while n <= K
        X = [X v];                              % Add the vertexes to the camera paths
        S = zeros(1,length(env.V));             % Initialize vertex intentions
        for r = randperm(R)
            v(r) = C(r).SEBS(env, R, Sa, L, M);       % Select next step
            % Share Intention
            view = env.A(env.Vmap(v(r)),:);
            view(1, env.Vmap(v(r))) = 1;
            Sa(logical(view)) = Sa(logical(view)) + 1; 
        end
        for r = 1:R
            C(r).V = v(r);                                     % Move
            viewed = sign(viewed + env.A(env.Vmap(v(r)),:)); 
            env.IvUpdate(t, C(r).V)                            % Update Idleness at time t at vertex C(r).V
            env.deheat(C(r).V);                                % Deheat viewed vertices
        end
        t = t + 1;
        n = n + 1;
        if sum(viewed) == length(viewed) && K==inf             % if all vertices have been viewed
            viewed = zeros(1,size(env.V,1));                   % reset viewed
            if t > 100
                K = n + K2;                                    % to stop the infinite loop
            end
        end
    end
    ViewTot(N) = sum(viewed)./length(env.V);                   % coverage percentage for the current run of the system
end
fprintf('Mean Coverage Percentage: %2.3f \n', mean(ViewTot));
