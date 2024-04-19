clear all;
close all;
clc;

% Recursive part of the distributed WLS can be embedded into 
% the description of the distributed KF

%% Human being position inside a room
% (suppose now to have all the sensors on moving robots 
% and still we want to estimate
% the human location

p = [3;4]; % we want to estimate this human/object of interest which is standing still in position (3,4)

%% Robot sensors readings measuring the human position (so moving objects collecting info)
n = 10; % number of robots (by increasing n if we have the same time
% horizon we are reducing the uncertainty since we have more sources
% of info to be shared)

% Uncertainty on each sensor readings relative measurements for each robot
Ri = cell(1, n);
for i=1:n
    Ri{i} = 1e0*(rand(2,2)-0.5); % by reducing it, we will converge to p a little bit closer
    Ri{i} = Ri{i}*Ri{i}'; % standard cov matrix positive definite and symmetric
end

% Robots Sensor models (different for each robot and it is generic but is assumed to be linear) to measure the human location
Hi = cell(1, n);
for i=1:n
    Hi{i} = rand(2,2) - 0.5;
    %% pre-condition to carry out the distributed WLS
    while rank(Hi{i}) < 2 % checking if the human position is statically observable for each single robot
        % which means that with 1 single shot/measurement of the presence of human in
        % front of each robot we are able to retrieve its coordinates/position of human (ofc if we use just a single sensor readingsour uncertainty will be very large)
        Hi{i} = rand(2,2) - 0.5;
    end
end

% describing the dynamic for the robot

%% Robot dynamics (discretized one)

Dt = 0.1; % sampling time
t = 0:Dt:10; % time window/horizon: by increasing the number of iterations (i.e 30) so tuning t
% we will get some kind of drift: so the estimates becomes worse and worse
% as time passes by (see X error and Y error charts) since we are using an
% OL update of the location of each robot (go to line 80) so we go with
% predictions so it starts increasing and increasing.

% If we multiply by 20 line 80 the cov matrix on the measurements input
% uncertainties of the determination of the location of the robots.

% This is again due to the fact we are just using prediction so sooner or
% later we will experience this drift because our estimate is diverging
% from the actual since we are just using encoders

% initialize the robot position somewhere in the env
% initial robot positions (for each robot)
x = (rand(2, n)-0.5)*10; % robot state (uniformly distributed between -5 and 5 meters along x and y)

% % robots inputs/ control laws pre-computed (for each and every robot)
% u = cell(1, n);
% for i=1:n
%     u{i} = Dt*[rand(1) + sin(t).^i; rand(1) + sqrt(i)*cos(t)];
% end
% 

% Motion of the vehicles simulation
xStore = cell(1, n);
for i=1:n
    xStore{i}(:, 1) = x(:, i); % initialization of the initial locations of the robots
end

% for i=1:n
%     xStore{i}(:, 1) = x(:, i);
%     for j = 1:length(t)-1
%         xStore{i}(:, j+1) = xStore{i}(:, j) + u{i}(:, j); % i-th robot discrete dynamic
%     end
% end

% Inputs uncertainties for each robot (uncertainties we have in determining
% the actual inputs on each robot) on the robot actuation
Qi = cell(1, n);
for i=1:n
    Qi{i} = 1e0*(rand(2,2)-0.5); % by reducing it, we will converge to p a little bit closer
    Qi{i} = Qi{i}*Qi{i}'; % uncertainty on our sensors
end

% GPS measurements uncertainties for the robots
R_GPS = cell(1, n); % set of uncertainties/cov matrices for each of the 
                    % robot measurement GPS readings of the location of robots
                    % coming from GPS
for i=1:n
    R_GPS{i} = 1e0*(rand(2,2)-0.5); % by reducing it, we will converge to p a little bit closer 
    R_GPS{i} = R_GPS{i}*R_GPS{i}'; % uncertainty on our GPS sensors
end
%% Assuming to make relative measurements (so each robot measures/collects 
% the human position in its own RF which is always the case since they use
% lidar, radar, cameras, UWB sensors,... and all these measurements
% will be RELATIVE): SO WE have a map of what happens in the ENV wrt our own RF

% Communication range for each robot (max distance that i-th robot can communicate)
CR = 50; % [m] % the larger the more all the robots converge to same value in x and y errors
% and we do not have anymore the divergence problem

% estimate locations for each and every robot (ROBOT STATE ESTIMATE)
x_est = cell(1, n); % robot state estimate to compute the predictions
P_est = cell(1, n); % uncertainty matrix of each robot
for i=1:n
    x_est{i} = zeros(2, length(t)); % initialization of the initial estimate of each of the robot state
    x_est{i}(:, 1) = x(:, i); % WE KNOW exactly where the robot starts (Hypot)
    P_est{i} = zeros(2,2); % we perfectly know the initial location of the robot (perfectly) at t = 0 so there is no uncertainty of the robots locations
    % at the beginning = 0 which means that the info matrix = cov^-1 means
    % that the amount of info in the initial positions at the beginning is actually infinite
end

% Store the value of the estimates (estimated position for each robot in
% the team) = each robot will store the value that has at each time step
% for the estimated position of the human being (as time evolution)
p_est_distr = cell(1, n); % estimate given the WLS of the human position
p_est_distr_MH = cell(1, n);
for i=1:n
    p_est_distr{i} = zeros(2, length(t)); % initialization
    p_est_distr_MH{i} = zeros(2, length(t)); % initialization for MH
end


% Robot control gain
robotGAIN = 0.1;

% Cycling along the time/system dynamics
% we are assuming that the robots are moving and at each time step
% they are able to measure the location of the human being
for cT = 1:length(t)-1

    %% Robot motion control

    % Robot motion
    % robots inputs/ control laws computed on-line (for each and every
    % robot) such that each robot goes towards the human
    for i=1:n
        % initial control laws inputs given to each robot (coming from estimated
        % quantities so they are not actually perfect) to converge towards
        % the human actual position
        u{i}(:, cT) = robotGAIN*(p_est_distr{i}(:, cT) - x_est{i}(:, cT)); % position of human - estimated of i-th robot position = feedback

        xStore{i}(:, cT+1) = xStore{i}(:, cT) + u{i}(:, cT); % i-th robot discrete dynamic
        
        % with this kind of system we cannot proof AS but Ultimate
        % unboundness stability so we will close to the human but we may
        % never ever say to reach the human due to uncertainties.
    end
    
    % we can turn the problem of AS towards the location of human and
    % the term u{i}(:, cT) = robotGAIN*(p_est_distr{i}(:, cT) - x_est{i}(:,
    % cT)); will proove AS without noise:
    
    % so when we are close enough to the human instead of doing strange
    % stuff, let's just stop.:)

    % in particular, we need to compute the probability:
    % if we want to be within a certain thr from the human with a certain probability, we
    % can use our estimated cov matrix P_est to understand the probability
    % that our robot will be exactly around mean = estimated value and use
    % the probability to understand the thr.
    % so wheh that thr is reached our control laws have to be switched off.
    
    % so we solve the problem stochastically since we deal with RVs /uncertainties with a
    % certain probability level: so we can only give probabilitistic guarantees to converge
    % towards the human.

    % if we have a Gaussian, we know we can be in thr = +-sigma with a
    % probability of 66.27%,...


    % what is the level of uncertainty, if we have the idea of the estimate
    % of our variance that gives an idea of the estimation error we have,
    % we can use it to switch the simple problem of AS to a problem of
    % practical stability so I will converge within a certain neighboorhood
    % of the actual location with that probability

    % with u{i}(:, cT) inside xStore we are assuming our robots are moving
    % perfectly according to the control law computed on-line but this is
    % not the case since we have NL effect on the actuation, backlashes in
    % the mechanism,... but the uncertainty mvnrnd([0;0], Qi{i})' on the line 162
    % takes into account of everything (all possible problems) so that we
    % are moving exactly in that direction, that we have measured the 
    % input given to our system but those are not the inputs actually
    % given, ..

    % SO mvnrnd([0;0], Qi{i})' can model either the imperfect robot
    % actuation or the imperfect readings of the sensors on the 
    % actuators




    % in order to simulate the fact we don't know actually where the robot is
    % because we have uncertainty in the sys. robot dynamics and the fact
    % we are measuring relative position
    
    %% Robot KFs for the position of each robot
    % each robot uses the data that comes from the sensors of the robot
    % motors + GPS measurements such that they are able to estimate their
    % own position

    % - Predictions
    uUnc = zeros(2, n); % 2 inputs and n robots
    % - input measurements simulation for each robot (they come with some
    % uncertainty)
    for i=1:n
        % unknown sensor reading knowledge got by using sensors
        uUnc(:, i) = u{i}(:, cT) + mvnrnd([0;0], Qi{i})'; % for the cT-th input measurement
        % we apply some inputs and when we read what actually happens on
        % the robot that would be different
    end

    % - Compute the predictions for all the robots
    
    for i=1:n
        % estimation of the location of each robot inside the env
        x_est{i}(:, cT+1) = x_est{i}(:, cT) + uUnc(:, i); % dynamics is assumed to be known
        % x_est is expressed in the WRF
        P_est{i} = P_est{i} + Qi{i}; % = A*P_est*A' for the propagation of the covariances but since the system dynamic has a dynamics matrix A = Id then we have just = P_est
    end

    % To avoid that kind of divergence on the estimates 
    % of the robots locations

    % let's assume to use some additional external sensors in order to estimate 
    % the robot location
    
    % - Measurements update to build up a KF for all the robots
    for i=1:n
        % for each robot we are assuming we have a 
        % measurement of the robot position (i.e we have a GPS
        % that gives each robot its own location)
        z_GPS = xStore{i}(:, cT+1) + mvnrnd([0;0], R_GPS{i})'; % location of the i-th robot + 2D uncertainty (x, y outputs in the WRF)
        innov = z_GPS - x_est{i}(:, cT+1); % measurement of i-th robot location - predicted location of the i-th robot
        H_GPS = eye(2); % GPS model used (x and y)

        % Updated Kalman estimates
        S_inn = H_GPS*P_est{i}*H_GPS' + R_GPS{i}; % cov of innovation
        W = P_est{i}* H_GPS'*inv(S_inn); % kalman filter gain = predicted uncertainty/cov matrix
        % standard MVUE for Gaussian noises
        x_est{i}(:, cT+1) = x_est{i}(:, cT+1) + W*innov; % prediction correction = updated state estimates
        P_est{i} = (eye(2) - W*H_GPS)*P_est{i}; % covariance matrix update
        % so each robot uses its own data to update its own location
    end

    % P_est expresses the confidence we have on the robot location and
    % tells us what is the probability of having the robot exactly in x_est
    % IN this case we have a linear system, the uncertainty is Gaussian so
    % with P_est we are computing the probability of having the robot
    % withing the mean (x_est) of the Gaussian 2D RV

    %% Robot human measurements (set of measurements related to the human location
    % and carried out by each and every robot)
    % Robots relative measurements using the sensors we have
    s = zeros(2,n);
    for i=1:n
        % Human in robot RF (relative position between robot and human)
        p_new = p - xStore{i}(:, cT+1); % p (actual position which is the RF so in the simulation we know
        % exactly where human is) - robot actual location
        
        % Sensor measurements of the i-th robot about the relative position of human +
        % noise/uncertainty
        s(:, i) = Hi{i}*p_new + mvnrnd([0;0], Ri{i})'; % i-th robot is able to measure p_new using model Hi
        % here we are assuming we can measure x and y (quite improbable) to
        % have a sensor able to get the x and y human position unless we
        % use very informative, costy and computationally hungry
        % solutions(survaillance cameras, image processing algorithms,...)
        % otherwise if we use something lighter we can think of measuring
        % distances/ranges wrt human (i.e): 
        
        % in this case having / measuring the range/distance from the human for each sensor 
        % and not just x and y what is the solution we have to think about to implement
        % the distributed WLS? We can use trilateration
        % ofc we wanna estimate the position of human wrt a WRF which is (x, y).

        % we need first of all to define the model zi_bar = hi(x, y) = sqrt((x-xi)^2
        % + (y-yi)^2) + eps_i = output of i-th sensor = value we can read
        % from each sensor

        % first of all, if I give you the sensor we have to understand what
        % is the extent of the uncertainty we can inject in each and every
        % measurement: 
        % so after having eps_i we can share info.

        % we have an infrastructure so the i-th node/sensor
        % is located in [xi, yi]^WRF which is known!
        
        % if we need to implement a LSsolution I have to solve an
        % unconstrained (no constraints) Optimal (we are minimizing an
        % index) problem:
        % the optimal problem we have to solve is to find the best estimate
        % but the optimal problem we are solving with LS is NON-LINEAR
        % since h is non-linear.

        % if we have a minimization problem:
        % to solve it we can take the gradient/derivative of the cost index
        % and put it = 0 and take it as solution of the optimal problem =
        % minimum

        % to find the zeros of a generic NL scalar function we can use
        % Numerical methods (i.e Gauss-Newton-Euler method: starts with a point, approximate
        % with a linear derivative, find the solution / zero of the linear derivative
        % and then we keep going until the difference between 2 successive updates of
        % algorithm <= threshold since it is numerical we have numerical uncertainties so
        % we will never ever end in the zero so we live with some approximation level):
        % doing the same for the minimization problem of before we have the
        % solution to a generic matrix like NL LS problem and we can wait
        % it with uncertainties so we can find a solution to the NL WLS
        % problem: see slides for the solution!

        % if we want to solve this problem in a distributed way:
        % is the very same as did for NL case, the only thing we have to
        % remember is that we do not have to solve just 1 round of
        % consensus but multiple rounds depending on the solution we are
        % approximating.

        % In this case we are solving a linear problem:
        % is the same as computing a linear WLS in which we assume our
        % function is linear so we find out a zero for this linear
        % approximation and then we go for a linear WLS in that zero and
        % repeat. The only difference is that we have to solve a sequence
        % of linearized solutions (i.e ranging measurements: we solve the
        % linear problem, we have a first solution then we linearize the
        % solution in the new point and then we go getting a 2nd solution
        % and 3rd .....

        % the difference of squares of the distances that leads us to a
        % linear problem and this one way in which we can compute a first
        % order approximation of the NL problem (it's not applying Taylor)
        % so it is an approximated solution since when computing the
        % difference of distances squared we are squaring the uncertainties
        % so we are increasing it so we need to go with several consecutive
        % solutions.


        % to solve a NL WLS in a distributed way?
        % we need to apply consensus in x1 since we have a linearized
        % version of the problem then we reach a consensus, we compute a
        % new version of the problem and then we apply consensus and
        % repeat!!! So for solving the NL problem is like solving the
        % linear one but having multiple consensus in between.

        % the more general is the solution we are thinking and the more
        % coomplicated is the our problem ofc the more computation time is
        % needed or higher the # of data to share inside the NW:
        % the main idea though is exactly the same, collect data and share
        % data.

    end
    
    % mvnrnd([0;0], Ri{i})' means an uncertainty with 0 mean so this is a
    % 2D RV and COV matrix Ri
        
    % by doing relative measurements we need to talk the same language in
    % order to share info: so if we do not have the same RF to refer the
    % position measurements we cannot build up a distributed algorithm

    % so we need to localize each robot in the same RF so we need
    % to estimate the robot position ==> KF

    % what is the measurement that the robot has to use in order to
    % localize itself: at least we need another sensor, not just the one
    % used to measure the human position, but we need some others sources of
    % info ==> we can simply measure the inputs (issue about that: we can
    % only do PREDICTIONs so there is a drift issue)

    
    % to build a distributed algorithm, we need all robots to have the info
    % to expressed in the same RF

    % Transform the robot relative measurements into the 'world'
    % measurements RF (the common RF for all the robots locations)
    s_world = zeros(2, n);
    for i=1:n
        s_world(:, i) = s(:, i) + Hi{i}*x_est{i}(:, cT+1); % current relative noisy robot sensor reading s + estimated x robot location
    end

    % s_world corresponds to the World measurement of the human for each
    % robot

    % now each robot has the measurement of the human location wrt world RF

    % we need just to share info with the other robots computing the WLS

    % let's assume that each robot communicates only with the robots
    % within its communication range and set it equal among all the robots
    % otherwise we get a graph topology not symmetric so it means it
    % becomes a directed graph and so that means that all the results seen
    % does not apply  but we need to add some others stuff

    % Distances
    
    %% CONSENSUS

    % Initialization (of our distributed WLS) of each and every sensor for the MAX degree and MH
    % The distributed WLS among all the robots is used to determine what is the location of the
    % human
    F = cell(n, 1); % vector of matrices
    a = cell(n, 1); % vector of matrices
    F_MH = cell(n, 1); % vector of matrices
    a_MH = cell(n, 1); % vector of matrices
    for i=1:n
        zi = s_world(:,i); % sensor readings/measurement for the i-th sensor
        F{i} = Hi{i}'*inv(Ri{i} + Hi{i}*P_est{i}*Hi{i}')*Hi{i}; % i-th value of the vector F(i) for sensor number i
        % we need to sum up (Ri{i} + P_est{i}) because we are transforming
        % from the robot RF in which we have just the sensor uncertainty to
        % the RF of all the robots (WRF): in doing so we need to know our
        % own position which comes out from a localization algorithm which
        % means it is affected by uncertainty.

        % Ri{i} + Hi{i}*P_est{i}*Hi{i}' = uncertainty on the sensor
        % readings for the i-th robot

        % SO we need to take into account that the robot location from which I am
        % taking the measurements is not precise to an infinite extent but
        % it is affected by uncertainty per se so we need to add this additional term
        % of uncertainty into the estimation of the robot:
        % if we don't take it into account we are building an inconsistent
        % filter and this will reflect on the distributed algorithm.


        a{i} = Hi{i}'*inv(Ri{i} + Hi{i}*P_est{i}*Hi{i}')*zi; % i-th value of the vector a(i) for sensor number i
        F_MH{i} = Hi{i}'*inv(Ri{i} + Hi{i}*P_est{i}*Hi{i}')*Hi{i}; % i-th value of the vector F(i) for sensor number i
        a_MH{i} = Hi{i}'*inv(Ri{i} + Hi{i}*P_est{i}*Hi{i}')*zi; % i-th value of the vector a(i) for sensor number i
        % all nodes have their own values
    end

    % the estimation error times itself transpose is P = cov of uncertainty
    % on the robot location

    % if the estimation error and eps (measurement error of the relative sensor) are
    % uncorrelated

    % Ri is the uncertainty on the measurements
    
    %
    m = 10; % number of consensus protocol msg exchanges (the higher, the better the match between centralized and distributed solutions)
    
    % Consensus protocol algorithm on F and a (using Max Degree weightings (we take the max between all the degrees))
    %% CONSENSUS protocol: we have all the previous measurements and they are sharing info about what have been measured
    % the robots are moving, they are measuring the human being, they are
    % collecting info and they are sharing info among robots for 10 times
    % so carrying out 10 msgs and this process is repeatead in a loop

    % CONSENSUS protocol = move + measurements + share info

    for k=1:m % k is the time
        % Topology matrix A of our NW (which is changing at each iteration)
        A = zeros(n, n); % new generation of an adjacency matrix with different probability 
        % connectProb = 0.5; 
        for i=1:n
            for j=i+1:n
                % the CR will act on the def of A matrix so we need to
                % understand how many robots are closed to each robot i
                % A(i, j) = round(rand(1) - (0.5 - connectProb)); % rand(1) generates a random number between 0 and 1 with round it will be either 0 or 1
                
                % distances among all the robots from i-th robot 
                d = xStore{i}(:, cT+1) - xStore{j}(:, cT+1); % location of i-th robot - location of j-th robot
                A(i, j) = (sqrt(d'*d) <= CR); % we are assuming the communication is always bidirectional (j can transmit to i and receive from i if it is in between 5 meters to i)
                
                % norm = euclidean distance between robot i and j
            end
        end
        A = A + A'; % adjacency matrix (vector of distances)
    
        % Degree vector of all the nodes in the NW
        D = A*ones(n, 1);
    
        % Max Degree weighting
        Fstore = F;
        astore = a;
        for i=1:n % i-th node computations
            for j=1:n
                % if node j sends info to node i ==> element = 1
                if(A(i,j) == 1)
                    F{i} = F{i} + 1/(1+max(D))*(Fstore{j} - Fstore{i});
                    a{i} = a{i} + 1/(1+max(D))*(astore{j} - astore{i});
                end
            end
        end
    
        % Metropolis Hastings weighting
        Fstore = F_MH;
        astore = a_MH;
        for i=1:n % i-th node computations: takes its own F and updates it with the info coming from the j-th node
            for j=1:n
                % if node j sends info to node i ==> element = 1
                if(A(i,j) == 1)
                    F_MH{i} = F_MH{i} + 1/(1+max(D(i), D(j)))*(Fstore{j} - Fstore{i});
                    a_MH{i} = a_MH{i} + 1/(1+max(D(i), D(j)))*(astore{j} - astore{i});
                end
            end
        end
    end

    %% Estimates of the human locations ==> we can converge towards the desired locaton

    for i=1:n
        % F{i} = what i-th robot has reached after m iterations of the
        % distributed protocol
        p_est_distr{i}(:, cT+1) = inv(F{i})*a{i}; % estimated position of the human  for the i-th robot at time cT
        p_est_distr_MH{i}(:, cT+1) = inv(F_MH{i})*a_MH{i}; % estimated position of the human  for the i-th robot at time cT (MH weights)
    end
     % so if the human is moving, the robots make the measurements and try
    % to converge towards the actual position of the human
    
    % i-th robot shares info with the neighboors and then its estimate
end

%% Plots (we are carrying out a WLS at each time step so we don't use the previous data)

figure(1), clf, hold on;
LegS = {};

for i=1:n
    plot(xStore{i}(1, :), xStore{i}(2, :)); % plot the positions (x, y) of each of the robot
    LegS{end+1} = [ 'Robot ', num2str(i)];
end
legend(LegS, 'Location','best');
xlabel('x [m]'); ylabel('y [m]');
title('Distributed Robotic WLS (convergence towards the human location)');

figure(2), clf, hold on;
LegS = {};

for i=1:n
    plot(t, p_est_distr{i}(1, :) - p(1)); % estimate i-th robot for the human position for the x coordinate - actual x of human = estimation error
    LegS{end+1} = [ 'Robot ', num2str(i)];
end
legend(LegS, 'Location','best');
xlabel('t [s]'); ylabel('x [m]');
title('X Error');

figure(3), clf, hold on;
LegS = {};

for i=1:n
    plot(t, p_est_distr{i}(2, :) - p(2)); % estimate i-th robot for the human position for the x coordinate - actual x of human = estimation error
    LegS{end+1} = [ 'Robot ', num2str(i)];
end
legend(LegS, 'Location','best');
xlabel('t [s]'); ylabel('y [m]');
title('Y Error');

% the different estimates among the robot for the X and Y error is just one
% time evolution:
% this means that all the robots has reached the WLS so all of them has
% exactly the same idea about the location of the human

% by reducing the number of msgs exchanges per iteration/per time step instead we will notice some
% differences in the estimates: the robots are not able to reach the common
% consensus (they have still good ideas somehow but they have problems in
% determining the consensus per se')

% if we have that some robots have estimation errors in x or y which are way above or below
% the consensus average (0 estimation error) then it means that sometimes
% some robots is not able to communicate to anyone so they have a very
% noisy estimates which comes from the uncertainties on the encoders and on
% the relative measurements.
