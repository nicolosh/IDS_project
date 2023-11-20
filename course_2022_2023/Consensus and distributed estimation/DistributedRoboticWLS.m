clear all;
clc;

p = [3;4];

%% Robot sensor readings

n = 10;

% Uncertainty
Ri = cell(1,n);
for i=1:n
    Ri{i} = rand(2,2)-0.5;
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

% Robots inputs
u = cell(1,n);
for i=1:n
    u{i} = 1e-1*[rand(1) + sin(t).^i; rand(1) + sqrt(i)*cos(t)];
end

% Motion of the vehicle
x_Store = cell(1,n);
for i=1:n
    x_Store{i}(:,1) = x(:,i);
    for j=1:length(t)-1
        x_Store{i}(:,j+1) = x_Store{i}(:,j) + u{i}(:,j);
    end
end

% Input uncertainties
Qi = cell(1,n);
for i=1:n
    Qi{i} = rand(2,2)-0.5;
    Qi{i} = Qi{i}*Qi{i}';
end


%% Relative measurements

% Communication range
CR = 5; % [m]

% Robot state estimate
x_est = cell(1,n);
P_est = cell(1,n);
for i=1:n
    x_est{i} = zeros(2,length(t));
    x_est{i}(:,1) = x(:,i);
    P_est{i} = zeros(2,2);
end

% Cycling along the time
for cT=1:length(t)-1
    
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
    
    % Initialisation
    F = cell(n,1);
    a = cell(n,1);
    F_MH = cell(n,1);
    a_MH = cell(n,1);
    for i=1:n
        zi = s_World(:,i);
        F{i} = Hi{i}'*inv(Ri{i} + P_est{i})*Hi{i};
        a{i} = Hi{i}'*inv(Ri{i} + P_est{i})*zi;
        F_MH{i} = Hi{i}'*inv(Ri{i} + P_est{i})*Hi{i};
        a_MH{i} = Hi{i}'*inv(Ri{i} + P_est{i})*zi;
    end
    
end

% Number of consensus protocol msg exchanges
m = 1;

for k=1:m
    % Topology matrix
    A = zeros(n,n);
    ProbOfConnection = 0.5;
    for i=1:n
        for j=i+1:n
            A(i,j) = round(rand(1)-(0.5-ProbOfConnection));
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

%% Plot

figure(1), clf, hold on;
LegS = {};
for i=1:n
    plot(x_Store{i}(1,:), x_Store{i}(2,:));
    LegS{end+1} = ['Robot ', num2str(i)];
end
legend(LegS, 'Location', 'best');
xlabel('x [m]'); ylabel('y [m]');


