clear all;
close all;
clc;

%% Human being position inside a room
p = [3;4]; % we want to estimate this

%% Sensors readings measuring the human position
n = 10; % 10 sensors

% Uncertainty
Ri = cell(1, n);
for i=1:n
    Ri{i} = rand(2,2)-0.5;
    Ri{i} = Ri{i}*Ri{i}';
end

% Sensor models
Hi = cell(1, n);
for i=1:n
    Hi{i} = rand(2,2) - 0.5;
    while rank(Hi{i}) < 2
        Hi{i} = rand(2,2) - 0.5;
    end
end

% %% Centralized WLS (collect all the measurements together and then we compute the
% % WLS solution p_hat)
% 
% H = [];
% R = []; % cov matrix of uncertainties of all the nodes and is diagonal since we assume no correlation between x and y measurements
% Z = [];
% for i=1:n
%     R = blkdiag(R, sigma_s(i)^2*eye(2));
%     H = [H; eye(2)];
%     Z = [Z; s(:, i)];
% end
% 
% % estimated human position using all 10 sensors at once
% p_hat = inv(H'*inv(R)*H)*H'*inv(R)*Z;

%% Multiple measurements

n_measure = 1000; % by increasing it, it should converge (as more evidence comes from the sensors the more
% we approach the actual values of the position of the human being)

% Initial estimate value
p_hat = [];
p_hat_store = zeros(2, n_measure);

% to measure the position
for cN = 1:n_measure
    s = zeros(2,n);
    for i=1:n
        % Sensor
        s(:, i) = Hi{i}*p + mvnrnd([0;0], Ri{i})';
    end

    %% Recursive centralized WLS
    H = [];
    R = [];
    Z = [];
    for i=1:n
        R = blkdiag(R, Ri{i});
        H = [H; Hi{i}];
        Z = [Z;  s(:, i)];
    end

    if isempty(p_hat)
        P_central_rec = inv(H'*inv(R)*H);
        p_hat = P_central_rec*H'*inv(R)*Z;
    else
        % Bayesian like recursive estimation
        S = H*P_central_rec*H' + R;
        W = P_central_rec*H'*inv(S);
        p_hat = p_hat + W*(Z - H*p_hat);
        P_central_rec = (eye(length(p_hat)) - W*H)*P_central_rec;
    end

    p_hat_store(:, cN) = p_hat;

end

%% Recursive Distributed WLS 
% initialization of each and every sensor
F = cell(n, 1); % vector of matrices
a = cell(n, 1); % vector of matrices
F_MH = cell(n, 1); % vector of matrices
a_MH = cell(n, 1); % vector of matrices
for i=1:n
    zi = s(:,i); % sensor readings/measurement for the i-th sensor
    F{i} = Hi{i}'*inv(Ri{i})*Hi{i}; % i-th value of the vector F(i) for sensor number i
    a{i} = Hi{i}'*inv(Ri{i})*zi; % i-th value of the vector a(i) for sensor number i
    F_MH{i} = Hi{i}'*inv(Ri{i})*Hi{i}; % i-th value of the vector F(i) for sensor number i
    a_MH{i} = Hi{i}'*inv(Ri{i})*zi; % i-th value of the vector a(i) for sensor number i
    % all nodes have their own values
end

%
m = 1; % number of consensus protocol msg exchanges (the higher, the better the match between centralized and distributed solutions)

% Consensus protocol algorithm on F and a (using Max Degree weightings (we take the max between all the degrees))

for k=1:m % k is the time
    % Topology matrix A of our NW (which is changing at each iteration)
    A = zeros(n, n); % new generation of an adjacency matrix with different probability 
    connectProb = 0.5; 
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

Estimates
p_hat_distr = zeros(length(p), n); % vector of estimates that each of the node has
for i=1:n
    p_hat_distr(:, i) = inv(F{i})*a{i};
end

p_hat_distr_MH = zeros(length(p), n); % vector of estimates that each of the node has
for i=1:n
    p_hat_distr_MH(:, i) = inv(F_MH{i})*a_MH{i};
end

%% Plots

%% Maximum degree

figure(1), clf, hold on;
plot(p(1), p(2), 'ro','LineWidth',2,'MarkerSize',10);
plot(p_hat(1), p_hat(2),'kd','LineWidth',2,'MarkerSize',10);
% for i=1:n
%     plot(p_hat_distr(1,i), p_hat_distr(2,i), 'gs','LineWidth',2,'MarkerSize',10);
% end
% for i=1:n
%     plot(s(1,i), s(2,i), 'bx','LineWidth',2,'MarkerSize',10);
% end
% legend('human actual position','Centralized WLS human estimated position','Distributed WLS human estimated position','measurements sensors','Location', 'best');
% xlabel('x [m]'); ylabel('y [m]');
% title('Maximum degree');

% by increasing n (sensor readings), p_hat will be much closer to the
% actual position p
% we can see that the distributed = centralized solution so the centralized
% solution of all the nodes is similar to the centralized version

%% Metropolis-Hastings

% figure(2), clf, hold on;
% plot(p(1), p(2), 'ro','LineWidth',2,'MarkerSize',10);
% plot(p_hat(1), p_hat(2),'kd','LineWidth',2,'MarkerSize',10);
% for i=1:n
%     plot(p_hat_distr_MH(1,i), p_hat_distr_MH(2,i), 'gs','LineWidth',2,'MarkerSize',10);
% end
% for i=1:n
%     plot(s(1,i), s(2,i), 'bx','LineWidth',2,'MarkerSize',10);
% end
% legend('human actual position','Centralized WLS human estimated position','Distributed WLS human estimated position','measurements sensors','Location', 'best');
% xlabel('x [m]'); ylabel('y [m]');
% title('Metropolis-Hastings');

% as we can notice MH is faster than MAX degree:
% for a small number of msgs in MAX degree the estimates are spread away from the
% actual global WLS solution while for MH they are very concentrated so
% close to global WLS solution


figure(3), clf, hold on;
plot(1:n_measure, p(1)*ones(1, n_measure),'r--');
plot(1:n_measure, p_hat_store(1, :),'b');
plot(1:n_measure, p(2)*ones(1, n_measure),'k--');
plot(1:n_measure, p_hat_store(2, :),'g');
legend('actual value for human position x = 3 coordinate','estimates at each iteration of x coord of human','actual value for human position y = 4 coordinate','estimates at each iteration of y coord of human','Location','Best');

