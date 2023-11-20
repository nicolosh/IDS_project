clear all;
clc;

p = [3;4];

%% Sensor readings

n = 10;

% Uncertainty
sigma_s = 0.5*ones(n,1);

s = zeros(2, n);
for i=1:n
    % Sensor
    s(:,i) = p + randn(2,1)*sigma_s(i);
end


%% Centralised WLS

H = [];
R = [];
Z = [];
for i=1:n
    R = blkdiag(R, sigma_s(i)^2*eye(2));
    H = [H; eye(2)];
    Z = [Z; s(:,i)];
end

p_hat = inv(H'*inv(R)*H)*H'*inv(R)*Z;


%% Distributed WLS

% Initialisation
F = cell(n,1);
a = cell(n,1);
F_MH = cell(n,1);
a_MH = cell(n,1);
for i=1:n
    Hi = eye(2);
    Ri = sigma_s(i)^2*eye(2);
    zi = s(:,i);
    F{i} = Hi'*inv(Ri)*Hi;
    a{i} = Hi'*inv(Ri)*zi;
    F_MH{i} = Hi'*inv(Ri)*Hi;
    a_MH{i} = Hi'*inv(Ri)*zi;
end

% Number of consensus protocol msg exchanges
m = 4;

for k=1:m-1
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

% Estimates
p_hat_Distr = zeros(length(p), n);
for i=1:n
    p_hat_Distr(:,i) = inv(F{i})*a{i};
end

% Estimates
p_hat_Distr_MH = zeros(length(p), n);
for i=1:n
    p_hat_Distr_MH(:,i) = inv(F_MH{i})*a_MH{i};
end

%% Plot

% Maximum degree
figure(1), clf, hold on;
plot(p(1), p(2) , 'ro', 'LineWidth', 2, 'MarkerSize', 10);
plot(p_hat(1), p_hat(2) , 'kd', 'LineWidth', 2, 'MarkerSize', 10);
for i=1:n
    plot(p_hat_Distr(1,i), p_hat_Distr(2,i), 'gs', 'LineWidth', 2, 'MarkerSize', 10);
end
for i=1:n
    plot(s(1,i), s(2,i), 'bx', 'LineWidth', 2, 'MarkerSize', 10);
end
legend('Actual p', 'Centralised WLS', 'Distributed WLS', 'Location', 'best');
xlabel('x [m]'); ylabel('y [m]');
title('Maximum Degree');

% Metropolis-Hastings
figure(2), clf, hold on;
plot(p(1), p(2) , 'ro', 'LineWidth', 2, 'MarkerSize', 10);
plot(p_hat(1), p_hat(2) , 'kd', 'LineWidth', 2, 'MarkerSize', 10);
for i=1:n
    plot(p_hat_Distr_MH(1,i), p_hat_Distr_MH(2,i), 'gs', 'LineWidth', 2, 'MarkerSize', 10);
end
for i=1:n
    plot(s(1,i), s(2,i), 'bx', 'LineWidth', 2, 'MarkerSize', 10);
end
legend('Actual p', 'Centralised WLS', 'Distributed WLS', 'Location', 'best');
xlabel('x [m]'); ylabel('y [m]');
title('Metropolis-Hastings');


