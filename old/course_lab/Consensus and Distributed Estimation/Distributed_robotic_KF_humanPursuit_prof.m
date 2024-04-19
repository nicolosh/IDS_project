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

%% Human kinematic model (is actually moving inside the environment) ==> (added for distributed KF)
% he follows more or less the same kind of dynamics we have defined for the
% robots
Ah = eye(2); % human dynamics which comes from the previous knowledge we have on human being motion
% Sequence of inputs for the human being
Bh = eye(2); % 2 indipendent inputs vector: one on x and one on y

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

% Communication range for each robot (max distance that i-th robot can communicate with the others)
CR = 50; % [m] % the larger the more all the robots converge to same value in x and y errors
% and we do not have anymore the divergence problem

% Sensing range (so each robot is able to sense a human if it is not closer
% than a certain value) can BE ADDED (remember that usually the sensing
% range is smaller than the CR)

% Human being state evolution (added for distributed KF)
pStore = zeros(2, length(t));
pStore(:, 1) = p; % initial location of the human

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
p_est_distr_KF = cell(1, n);

% we need also to have an idea of the initial position of the human
% wrt actual position

% Uncertainty on the initial location of the human being
Ph_KF = cell(1, n);

for i=1:n
    p_est_distr{i} = zeros(2, length(t)); % initialization
    p_est_distr_MH{i} = zeros(2, length(t)); % initialization for MH
    p_est_distr_KF{i} = zeros(2, length(t)); % initialization for the distributed KF
    
    % if we assume that the human at the beginning will be in a region
    % 10x10 [m^2] we want to find out 99% probability of being there
    
    Ph_KF{i} = (10/3)^2*eye(2); % divide by 3 since we want 3 sigma
    % it expresses the ignorance about the location of the human apart from
    % the fact that at the beginning it should be with a coordinate x and y
    % between 0 and 10, 0 and 10.
    
end

% Uncertainty in the human being motion so on the inputs uHuman since we
% don't know the actual values of uHuman
Qh = (5e-2)^2*eye(2); % cov matrix assuming to be isotropic on x and y

% by defining Qh in that way we are assuming that the robot
% in 100 ms (Dt) actually travels for +- 3 [m] with 99% probability:
% we need to understand where the human could be after Dt:
% if p is the location of the human, we need to understand where 
% it could be around p in Dt so we need to have an idea of the range
% in which the human could be at the next time step with a certain
% probability:
% to be able to identify the human inside a certain range with 99% 
% of probability, if we assume that the human is moving according
% to a 2D Gaussian pdf/randon walk with indipendent increment on x and y, we are
% assuming that the robot moves at most at 3 sigma = 15 cm if 
% we assume to walk at 1.5 [m/s] so means that sigma = 5 cm and
% sigma^2 = 25 cm^2

% so we don't know where it moves but we can have an upper bound
% 

% Robot control gain
robotGAIN = 0.5; % to be changed if the robots are a bit too slow to keep up
                 % the velocity of the human being

% Human input
uHuman = [0.15; 0.1]; % [m / 100 [ms]]

% probability of changing human direction
probDirH = 0.8;

% Cycling along the time/system dynamics
% we are assuming that the robots are moving and at each time step
% they are able to measure the location of the human being
for cT = 1:length(t)-1
    
    %% Human being dynamics (added for distributed KF) = human motion
    % let us change the motion of the human here and there
    % we can use a MP so a markov chain describing human motion

    % we see if the human is changing direction or not
    if rand(1) < probDirH
        uHuman = (rand(2, 1) - 0.5)*0.3; % since the max step was 15 cm: we generated 2 indipendent inputs in between +- 15 [cm]
    end
    pStore(:, cT+1) = Ah*pStore(:, cT) + Bh*uHuman;

    %% Robot motion control

    % Robot motion
    % robots inputs/ control laws computed on-line (for each and every
    % robot) such that each robot goes towards the human
    for i=1:n
        % initial control laws inputs given to each robot (coming from estimated
        % quantities so they are not actually perfect) to converge towards
        % the human actual position
        u{i}(:, cT) = robotGAIN*(p_est_distr_KF{i}(:, cT) - x_est{i}(:, cT)); % position of human - estimated of i-th robot position = feedback

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
        p_new = pStore(:, cT+1) - xStore{i}(:, cT+1); % pStore is the measuremnet we are doing about
        % human location (actual human position which is the RF so in the simulation we know
        % exactly where human is) - robot actual location
        
        % Sensor measurements of the i-th robot about the relative position of human +
        % noise/uncertainty
        s(:, i) = Hi{i}*p_new + mvnrnd([0;0], Ri{i})'; % i-th robot is able to measure p_new using model Hi
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
    
    %% CONSENSUS - Distributed KF

    % - Prediction step for all the robots
    for i=1:n
        % each robot carries out its own prediction
        % =========================================
        % we need A matrix, we don't have access to the inputs but 
        % we can model them as a random walk so we need to setup 
        % the cov matrix Q of uncertainty to model the fact we do not know 
        % what is the direction that the human will take
        p_est_distr_KF{i}(:, cT+1) = Ah*p_est_distr_KF{i}(:, cT);
        Ph_KF{i} = Ah*Ph_KF{i}*Ah' + Qh; % with Qh expressing our ignorance about the human motion
    end
       
    % if we keep going with prediction model after few time steps
    % we do not know anything since the cov matrix is huge

    % - Update ==> we need to define ai and Fi

    % Initialization (of our distributed WLS) of each and every sensor for the MAX degree and MH
    % The distributed WLS among all the robots is used to determine what is the location of the
    % human
    F_MH = cell(n, 1); % vector of matrices
    a_MH = cell(n, 1); % vector of matrices
    for i=1:n
        zi = s_world(:,i); % sensor readings/measurement for the i-th sensor
        F_MH{i} = Hi{i}'*inv(Ri{i} + Hi{i}*P_est{i}*Hi{i}')*Hi{i}; % i-th value of the vector F(i) for sensor number i
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


        a_MH{i} = Hi{i}'*inv(Ri{i} + Hi{i}*P_est{i}*Hi{i}')*zi; % i-th value of the vector a(i) for sensor number i
        % all nodes have their own values
    end

    % P_est{i} = uncertainty on the robot location:
    % we need to transform the relative into absolute measurements
    % otherwise we cannot carry out the distributed estimation approach

    % the estimation error times itself transpose is P = cov of uncertainty
    % on the robot location

    % if the estimation error and eps (measurement error of the relative sensor) are
    % uncorrelated

    % Ri is the uncertainty on the measurements
    
    %%
    m = 1; % number of consensus protocol msg exchanges (the higher, the better the match between centralized and distributed solutions)
    % if we decrease m to 1 or 2 for example the algorithm still works
    % because on the measurement we are sharing info even though we do not
    % give the filter the chance to reach a consensus but just making 1
    % share of info and go ahead.

    % Still this kind of approach inherites the properties we were
    % discussing so the estimate will fall within the convex hull of the
    % previous estimate so even if we share just 1 msg we wil get closer to
    % the actual estimate/value so with just 1 link of connection among
    % nodes.


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
                % if we are within/below that distance we can talk otherwise no
                A(i, j) = (sqrt(d'*d) <= CR); % we are assuming the communication is always bidirectional (j can transmit to i and receive from i if it is in between 5 meters to i)
                
                % norm = euclidean distance between robot i and j
            end
        end
        A = A + A'; % adjacency matrix (vector of distances)
    
        % Degree vector of all the nodes in the NW
        D = A*ones(n, 1);
        
        % Metropolis-Hastings weighting
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

    % - Measurement update (to be carried out for all the robots) ---
    % all robots will use F and and then it will try to update the
    % predicted value p_est_distr_KF (see pag. 33 for formulas)
    for i=1:n
        predP = Ph_KF{i};
        Ph_KF{i} = inv(Ph_KF{i} + n*F_MH{i}); % updated P = predicted P(k+1)^- + n_elements/robots with which we are are sharing info * Fi
        p_est_distr_KF{i}(:, cT+1) = Ph_KF{i}*(predP*p_est_distr_KF{i}(:, cT+1) + n*a_MH{i}); % p_est_distr_KF{i}(:, cT) = predicted state
    end

end

%% Plots (we are carrying out a WLS at each time step so we don't use the previous data)

figure(1), clf, hold on;
LegS = {};

for i=1:n
    plot(xStore{i}(1, :), xStore{i}(2, :)); % plot the positions (x, y) of each of the robot
    LegS{end+1} = [ 'Robot ', num2str(i)];
end
plot(pStore(1, :), pStore(2,:),'k','LineWidth', 2);
LegS{end+1} = ['Human'];
legend(LegS, 'Location','best');
xlabel('x [m]'); ylabel('y [m]');
title('Distributed Robotic WLS (convergence towards the human location)');

figure(2), clf, hold on;
LegS = {};

for i=1:n
    plot(t, p_est_distr_KF{i}(1, :) - pStore(1,:)); % estimate i-th robot for the human position for the x coordinate - actual x of human = estimation error
    LegS{end+1} = [ 'Robot ', num2str(i)];
end
legend(LegS, 'Location','best');
xlabel('t [s]'); ylabel('x [m]');
title('X Error');

figure(3), clf, hold on;
LegS = {};

for i=1:n
    plot(t, p_est_distr_KF{i}(2, :) - pStore(2,:)); % estimate i-th robot for the human position for the x coordinate - actual x of human = estimation error
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
