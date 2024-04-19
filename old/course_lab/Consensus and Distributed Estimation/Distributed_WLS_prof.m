clear all;
close all;
clc;

%% Human being position inside a room
p = [3;4]; % we want to estimate this

%% Sensors readings measuring the human position
n = 10; % 10 sensors

% Uncertainty in sensors readings
sigma_s = 0.5*ones(n,1);


s = zeros(2, n); % sensors
for i=1:n
    % Sensors readings + Gaussian uncertainty (0 mean (unbiased sensors))
    s(:, i) = p + randn(2,1)*sigma_s(i);
end


%% Centralized WLS (collect all the measurements together and then we compute the
% WLS solution p_hat)

H = [];
R = []; % cov matrix of uncertainties of all the nodes and is diagonal since we assume no correlation between x and y measurements
Z = [];
for i=1:n
    R = blkdiag(R, sigma_s(i)^2*eye(2));
    H = [H; eye(2)];
    Z = [Z; s(:, i)];
end

% estimated human position using all 10 sensors at once
p_hat = inv(H'*inv(R)*H)*H'*inv(R)*Z;


%% Distributed WLS

% initialization of each and every sensor
F = cell(n, 1); % vector of matrices
a = cell(n, 1); % vector of matrices
F_MH = cell(n, 1); % vector of matrices
a_MH = cell(n, 1); % vector of matrices
for i=1:n
    Hi = eye(2); % i-th sensor model = measurement of x, y together for the measurement of each sensor
    Ri = sigma_s(i)^2*eye(2); % cov matrix of the measurement for the i-th sensor = i-th sensor uncertainty
    zi = s(:,i); % sensor readings/measurement for the i-th sensor
    F{i} = Hi'*inv(Ri)*Hi; % i-th value of the vector F(i) for sensor number i
    a{i} = Hi'*inv(Ri)*zi; % i-th value of the vector a(i) for sensor number i
    F_MH{i} = Hi'*inv(Ri)*Hi; % i-th value of the vector F(i) for sensor number i
    a_MH{i} = Hi'*inv(Ri)*zi; % i-th value of the vector a(i) for sensor number i

    % all nodes have their own values
end

% Topology matrix A of our NW (FIXED topology generation)
A = zeros(n, n);
connectProb = 0.5; % was 0.1
for i=1:n
    for j=i+1:n
        A(i, j) = round(rand(1) - (0.5 - connectProb)); % rand(1) generates a random number between 0 and 1 with round it will be either 0 or 1
    end
end
% we are pushing the probability of having a connection (in order to reduce the amount of connections) that is 
% less than 50%: so when the probability of connection decreases so drops
% down to 0.1 then we are removing 0.4 from a number that is between 0 and 1 
% then we compute the round so we are reducing the probability

A = A + A'; % adjacency matrix

% Degree vector of all the nodes in the NW
D = A*ones(n, 1);

%%
m = 4; % number of consensus protocol msg exchanges (the higher, the better the match between centralized and distributed solutions)

%% Consensus protocol algorithm on F and a (using Max Degree weightings (we take the max between all the degrees))

for k=1:m-1 % k is the time
    % Topology matrix A of our NW (which is changing at each iteration)
    A = zeros(n, n); % new generation of an adjacency matrix with different probability 
    connectProb = 0.5; % was 0.1
    for i=1:n
        for j=i+1:n
            A(i, j) = round(rand(1) - (0.5 - connectProb)); % rand(1) generates a random number between 0 and 1 with round it will be either 0 or 1
        end
    end
    A = A + A'; % adjacency matrix

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
    for i=1:n % i-th node computations
        for j=1:n
            % if node j sends info to node i ==> element = 1
            if(A(i,j) == 1)
                F_MH{i} = F_MH{i} + 1/(1+max(D(i), D(j)))*(Fstore{j} - Fstore{i});
                a_MH{i} = a_MH{i} + 1/(1+max(D(i), D(j)))*(astore{j} - astore{i});
            end
        end
    end
end

% after 20 msgs exchange, we will compute the estimate (each node will have
% its own estimate)
p_hat_distr = zeros(length(p), n); % vector of estimates that each of the node has
for i=1:n
    p_hat_distr(:, i) = inv(F{i})*a{i};
end

%% Consensus protocol algorithm on F and a (using Metropolis Hastings weightings)
% with this weighting approach we need to take the max between i-th node
% and j-th node degree

% for k=1:m-1 % k is the time
%     % Metropolis Hastings weighting
%     Fstore = F;
%     astore = a;
%     for i=1:n % i-th node computations
%         for j=1:n
%             % if node j sends info to node i ==> element = 1
%             if(A(i,j) == 1)
%                 F{i} = F{i} + 1/(1+max(D(i), D(j)))*(Fstore{j} - Fstore{i});
%                 a{i} = a{i} + 1/(1+max(D(i), D(j)))*(astore{j} - astore{i});
%             end
%         end
%     end
% end

% after 20 msgs exchange, we will compute the estimate (each node will have
% its own estimate)
p_hat_distr_MH = zeros(length(p), n); % vector of estimates that each of the node has
for i=1:n
    p_hat_distr_MH(:, i) = inv(F_MH{i})*a_MH{i};
end

%% Plots

%% Maximum degree

figure(1), clf, hold on;
plot(p(1), p(2), 'ro','LineWidth',2,'MarkerSize',10);
plot(p_hat(1), p_hat(2),'kd','LineWidth',2,'MarkerSize',10);
for i=1:n
    plot(p_hat_distr(1,i), p_hat_distr(2,i), 'gs','LineWidth',2,'MarkerSize',10);
end
for i=1:n
    plot(s(1,i), s(2,i), 'bx','LineWidth',2,'MarkerSize',10);
end
legend('human actual position','Centralized WLS human estimated position','Distributed WLS human estimated position','measurements sensors','Location', 'best');
xlabel('x [m]'); ylabel('y [m]');
title('Maximum degree');

% by increasing n (sensor readings), p_hat will be much closer to the
% actual position p
% we can see that the distributed = centralized solution so the centralized
% solution of all the nodes is similar to the centralized version

%% Metropolis-Hastings

figure(2), clf, hold on;
plot(p(1), p(2), 'ro','LineWidth',2,'MarkerSize',10);
plot(p_hat(1), p_hat(2),'kd','LineWidth',2,'MarkerSize',10);
for i=1:n
    plot(p_hat_distr_MH(1,i), p_hat_distr_MH(2,i), 'gs','LineWidth',2,'MarkerSize',10);
end
for i=1:n
    plot(s(1,i), s(2,i), 'bx','LineWidth',2,'MarkerSize',10);
end
legend('human actual position','Centralized WLS human estimated position','Distributed WLS human estimated position','measurements sensors','Location', 'best');
xlabel('x [m]'); ylabel('y [m]');
title('Metropolis-Hastings');

% as we can notice MH is faster than MAX degree:
% for a small number of msgs in MAX degree the estimates are spread away from the
% actual global WLS solution while for MH they are very concentrated so
% close to global WLS solution
