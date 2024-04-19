clear all;
clc;

p = [3;4];

%% Robot sensor readings

n = 10;

% Uncertainty
Ri = cell(1,n);
for i=1:n
    Ri{i} = 1e-3*(rand(2,2)-0.5);
    Ri{i} = Ri{i}*Ri{i}';
end

% Sensor models
Hi = cell(1,n);
for i=1:n
    Hi{i} = rand(2,2)-0.5;
    while rank(Hi{i}) < 2
        Hi{i} = rand(2,2)-0.5;
    end
end

%% Robot dynamics

Dt = 0.1;
t = 0:Dt:10;

% Initial robot positions
x = (rand(2,n)-0.5)*10;

% Motion of the vehicle
x_Store = cell(1,n);
for i=1:n
    x_Store{i}(:,1) = x(:,i);
end

% Input uncertainties
Qi = cell(1,n);
for i=1:n
    Qi{i} = 1e-3*(rand(2,2)-0.5);
    Qi{i} = Qi{i}*Qi{i}';
end

% GPS measurement uncertainties for the robots
R_GPS = cell(1,n);
for i=1:n
    R_GPS{i} = 1e-3*(rand(2,2)-0.5);
    R_GPS{i} = R_GPS{i}*R_GPS{i}';
end



%% Relative measurements

% Communication range
CR = 50; % [m]

% Robot state estimate
x_est = cell(1,n);
P_est = cell(1,n);
for i=1:n
    x_est{i} = zeros(2,length(t));
    x_est{i}(:,1) = x(:,i);
    P_est{i} = zeros(2,2);
end

% Storing the estimates
p_est_distr = cell(1,n);
p_est_distr_MH = cell(1,n);
for i=1:n
    p_est_distr{i} = zeros(2,length(t));
    p_est_distr_MH{i} = zeros(2,length(t));
end

% Robot control gain
RobGain = 0.1;

% Cycling along the time
for cT=1:length(t)-1

    % Robot motion
    for i=1:n
        u{i}(:,cT) = RobGain*(p_est_distr{i}(:,cT) - x_est{i}(:,cT));
        x_Store{i}(:,cT+1) = x_Store{i}(:,cT) + u{i}(:,cT);
    end

    
    %% Robot Kalman filters
    
    % Predictions
    % - Robot input measurements
    uUnc = zeros(2,n);
    for i=1:n
        uUnc(:,i) = u{i}(:,cT) + mvnrnd([0;0], Qi{i})';
    end
    % - Compute the predictions
    for i=1:n
        x_est{i}(:,cT+1) = x_est{i}(:,cT) + uUnc(:,i);
        P_est{i} = P_est{i} + Qi{i};
    end
    % Mesurements update
    for i=1:n
        z_GPS = x_Store{i}(:,cT+1) + mvnrnd([0;0], R_GPS{i})';
        Innovation = z_GPS - x_est{i}(:,cT+1);
        H_GPS = eye(2);
        % Updated Kalman estimates
        S_Inno = H_GPS*P_est{i}*H_GPS' + R_GPS{i}; % Covariance of Innovation
        W = P_est{i}*H_GPS'*inv(S_Inno); % Kalman filter gain
        x_est{i}(:,cT+1) = x_est{i}(:,cT+1) + W*Innovation; % Updated state estimate
        P_est{i} = (eye(2) - W*H_GPS)*P_est{i}; % Updated covariance matrix
    end
        
    %% Robot human measurments
    
    % Robots measurements
    s = zeros(2, n);
    for i=1:n
        % Human in robot reference frame
        p_new = p - x_Store{i}(:,cT+1);
        
        % Sensor
        s(:,i) = Hi{i}*p_new + mvnrnd([0;0], Ri{i})';
    end
    
    % Transform the robot measurements into world measurements (the common
    % reference frame for all the robots)
    s_World = zeros(2,n);
    for i=1:n
        s_World(:,i) = s(:,i) + Hi{i}*x_est{i}(:,cT+1);
    end
    
    %% Consensus
    
    % Initialisation
    F = cell(n,1);
    a = cell(n,1);
    F_MH = cell(n,1);
    a_MH = cell(n,1);
    for i=1:n
        zi = s_World(:,i);
        F{i} = Hi{i}'*inv(Ri{i} + Hi{i}*P_est{i}*Hi{i}')*Hi{i};
        a{i} = Hi{i}'*inv(Ri{i} + Hi{i}*P_est{i}*Hi{i}')*zi;
        F_MH{i} = Hi{i}'*inv(Ri{i} + Hi{i}*P_est{i}*Hi{i}')*Hi{i};
        a_MH{i} = Hi{i}'*inv(Ri{i} + Hi{i}*P_est{i}*Hi{i}')*zi;
    end
    
    % Number of consensus protocol msg exchanges
    m = 10;
    
    for k=1:m
        % Topology matrix
        A = zeros(n,n);
        for i=1:n
            for j=i+1:n
                d = x_Store{i}(:,cT+1) - x_Store{j}(:,cT+1);
                A(i,j) = (sqrt(d'*d) <= CR);
            end
        end
        A = A + A';
        
        % Degree vector
        D = A*ones(n,1);
        
        % Maximum Degree Waighting
        FStore = F;
        aStore = a;
        for i=1:n
            for j=1:n
                if A(i,j) == 1
                    F{i} = F{i} + 1/(1+max(D))*(FStore{j} - FStore{i});
                    a{i} = a{i} + 1/(1+max(D))*(aStore{j} - aStore{i});
                end
            end
        end
        
        % Metropolis-Hastings
        FStore = F_MH;
        aStore = a_MH;
        for i=1:n
            for j=1:n
                if A(i,j) == 1
                    F_MH{i} = F_MH{i} + 1/(1+max(D(i), D(j)))*(FStore{j} - FStore{i});
                    a_MH{i} = a_MH{i} + 1/(1+max(D(i), D(j)))*(aStore{j} - aStore{i});
                end
            end
        end
    end
    
    %% Estimates
    
    for i=1:n
        p_est_distr{i}(:,cT+1) = inv(F{i})*a{i};
        p_est_distr_MH{i} = inv(F_MH{i})*a_MH{i};
    end
end

%% Plot

figure(1), clf, hold on;
LegS = {};
for i=1:n
    plot(x_Store{i}(1,:), x_Store{i}(2,:));
    LegS{end+1} = ['Robot ', num2str(i)];
end
legend(LegS, 'Location', 'best');
xlabel('x [m]'); ylabel('y [m]');


figure(2), clf, hold on;
LegS = {};
for i=1:n
    plot(t, p_est_distr{i}(1,:) - p(1));
    LegS{end+1} = ['Robot ', num2str(i)];
end
title('X Error');
legend(LegS, 'Location', 'best');
xlabel('t [s]'); ylabel('x [m]');

figure(3), clf, hold on;
LegS = {};
for i=1:n
    plot(t, p_est_distr{i}(2,:) - p(2));
    LegS{end+1} = ['Robot ', num2str(i)];
end
title('Y Error');
legend(LegS, 'Location', 'best');
xlabel('t [s]'); ylabel('y [m]');
