clear all
close all
clc

%% Network topology
% Number of nodes
n = 100;

% Adjacency 
Prob = 0.9;
Aq = zeros(n,n);
for i =1:n
    for j = i+1:n
        if rand(1) > Prob
            Aq(i,j) = 1;
        end
    end
end
Aq = Aq + Aq';

% Weights of the consesus algorithm
D = sum(Aq,2);
Q = zeros(n,n);

for i = 1:n
    for j = 1:n
        if not(i==j) && (Aq(i,j) == 1)
            Q(i,j) = 1/(max(D(i),D(j)) + 1);
        end
    end
    Q(i,i) = 1 - sum(Q(i,:));
end

%% Simulation set-up
dt = 0.1;
t = 0:dt:5;

% System to estimate
x = 12;

%% Sensors

% Sensors uncertainties
sigma_z = zeros(n,1);
for i = 1:n
    sigma_z(i) = i*0.1;
end

z = zeros(n,1);
for i =1:n
    z(i) = x + randn(1)*sigma_z(i);
end

%% Simulation

a = zeros(n,1);
b = zeros(n,1);

a_new = zeros(n,1);
b_new = zeros(n,1);

% Node initialisation
for i =1:n
    a(i) = z(i)/sigma_z(i)^2;
    b(i) = 1/sigma_z(i)^2;
end

% Message exchange
x_est = zeros(n,length(t)+1);
x_est(:,1) = z;

for i = 1:length(t)
    % Adjacency
    Aq = zeros(n,n);
    for ii = 1:n
        for j = ii+1:n
            if rand(1) > Prob
                Aq(ii,j) = 1;
            end
        end
    end
    Aq = Aq + Aq';
    % Weights of the consesus algorithm
    D = sum(Aq,2);
    Q = zeros(n,n);

    for ii = 1:n
        for j = 1:n
            if not(ii==j) && (Aq(ii,j) == 1)
                Q(ii,j) = 1/(max(D(ii),D(j)) + 1);
            end
        end
        Q(ii,ii) = 1 - sum(Q(ii,:));
    end

    for j = 1:n
        a_new(j) = a(j);
        b_new(j) = b(j);

        for k = 1:n
            if not(Aq(j,k)== 0)
                a_new(j) = a_new(j) + Q(j,k)*(a(k) - a(j));
                b_new(j) = b_new(j) + Q(j,k)*(b(k) - b(j));
            end
        end
    end
    
    % Store data
    a  = a_new;
    b = b_new;
    for j=1:n
        x_est(j,i+1) = a(j)/b(j); 
    end
end

% Plot
figure(1), clf, hold on;
LegS = cell(1,n+1);

for i =1:n
    plot(1:length(t)+1, x_est(i,:));
    LegS{i} = ['n_{',num2str(i),'}'];
end

plot(1:length(t)+1, x*ones(1,length(t)+1));

LegS{n+1} = 'Actual';
legend(LegS, 'Location','best')
xlabel('Step')






