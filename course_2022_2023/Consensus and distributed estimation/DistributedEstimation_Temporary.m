clear all;
close all;
clc;

%% Temperature

% Number of sensors
n = 10;

T = 22;

%% Three sensors readings

sigma_T = 0.2;

Ti = 22 + rand(n,1)*sigma_T;


%% Algorithm

% Tavg = zeros(n,1);
% for i=1:n
%     Tavg(i) = Ti(1)/3 + Ti(2)/3 + Ti(3)/3;
% end

Q2 = ones(n,n);
Q2 = 1/n*Q2;
Tavg2 = Q2*Ti;
Tavg2 = Q2*Tavg2;

% Topology matrix
A = zeros(n,n);
for i=1:n
    for j=i+1:n
        A(i,j) = round(rand(1));
    end
end
A = A + A';
% Degree vector
D = A*ones(n,1);
m = 20;
Tavg3 = zeros(n,m);
Tavg3(:,1) = Ti;
for k=1:m-1
    for i=1:n
        Tavg3(i,k+1) = Tavg3(i,k);
        for j=1:n
            if A(i,j) == 1
                Tavg3(i,k+1) = Tavg3(i,k+1) + 1/(1+max(D))*(Tavg3(j,k) - Tavg3(i,k));
            end
        end
    end
end

%% Plot

figure(1), clf, hold on;
% plot(1:n, Tavg - T)
plot(1:n, Tavg2 - T, 'r--')
plot(1:n, Tavg3 - T, 'k:')
xlabel('Sensor number')
ylabel('Temperature estimation error');


figure(2), clf, hold on;
for j=1:n
    plot(1:m, Tavg3(j,:));
end
plot(1:m, T*ones(1,m), 'r--');



