clear all;
close all;
clc;

% Number nodes
n = 3;

% Sensed quantity
T = 23;

% Sensor measurements
mu_s = 0;
sigma_s = 1e-2;

x = zeros(n,1);

for i = 1:n
    x(i) = T + randn(1)*sigma_s + mu_s;
end

%% Compute the mean

% Data sharing
x_new = zeros(1,n);

for i=1:n
    x_new(i) = x(i)/n;
    for j = 1:n
        if not(j==i)
            x_new(i) = x_new(i) + x(j)/n;
        end
    end
end

%% Consensus matrix
Q = 1/n*ones(n,1)*ones(n,1)';
Q(2,3) = 0;
Q(3,2) = 0;
Q(2,[1,2]) = 0.5;
Q(3,[1,3]) = 0.5;

x_new2 = Q*x;

x1 = rand(1);
y1 = min(max(0,1-x1 + rand(1)),1);
Q = [x1+y1-1, 1-x1, 1-y1;
    1-x1, x1 , 0;
    1-y1, 0, y1];

x_new3 = Q*x;



