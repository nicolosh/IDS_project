clear all;
close all;
clc;

%% Vehicle - 1

Dt = 0.1;
t = 0:Dt:100;

x1 = rand(1);

u1 = 1 + sin(t);

% Friction effect
a = -0.1;
b = Dt*1.2;

%% Vehicle - 2

x2 = 2 + rand(1);

u2 = 1 + cos(t);


%% Dynamic

x1Store = size(1,length(t));
x1Store(1) = x1;
x2Store = size(1,length(t));
x2Store(1) = x2;
for i=1:length(t)-1
    % Vehicle 1
    x1Store(i+1) = a*x1Store(i) + b*u1(i);
    % Vehicle 2
    x2Store(i+1) = a*x2Store(i) + b*u2(i);
end


%% KF tracker

% Uncertainty
% - Actual measured input
% Vehicle 1
mu_u1 = 0;
sigma_u1 = 0.1;
u1_bar = u1 + randn(1,length(u1))*sigma_u1 + mu_u1;
% Vehicle 2
mu_u2 = 0;
sigma_u2 = 0.05;
u2_bar = u2 + randn(1,length(u2))*sigma_u2 + mu_u2;
% - Actual measured GPS
% Vehicle 1
mu_gps1 = 0;
sigma_gps1 = 0.3;
x1GPS = x1Store + randn(1,length(x1Store))*sigma_gps1 + mu_gps1;
ProbGPS1 = 0.05;
% Vehicle 2
mu_gps2 = 0;
sigma_gps2 = 0.2;
x2GPS = x2Store + randn(1,length(x2Store))*sigma_gps2 + mu_gps2;
ProbGPS2 = 0.9;
% Relative radar measurements
mu_radar = 0;
sigma_radar = 0.1;
ProbRadar = 0.8;

% Kalman filter
% - Vehicle 1
x1Est = zeros(1, length(t));
P1 = 10^2;
P1Store = zeros(1, length(t));
P1PredStore = zeros(1, length(t));
% - Vehicle 2
x2Est = zeros(1, length(t));
P2 = 10^2;
P2Store = zeros(1, length(t));
P2PredStore = zeros(1, length(t));
% - Vehicle 1p2
x1p2Est = zeros(1, length(t));
P1p2 = 10^2;
P1p2Store = zeros(1, length(t));
P1p2PredStore = zeros(1, length(t));

for i=1:length(t)-1
    
    %% Vehicle 1
    
    % Prediction
    x1EstPred = a*x1Est(i) + b*u1_bar(i);
    P1pred = a*P1*a' + b*sigma_u1^2*b';
    
    % Update
    pGPS1 = rand(1);
    if (pGPS1 <= ProbGPS1)
        H = 1;
        InnCov = H*P1pred*H' + sigma_gps1^2;
        W = P1pred*H'*inv(InnCov);
        x1Est(i+1) = x1EstPred + W*(x1GPS(i+1) - H*x1EstPred);
        P1 = (eye(length(x1)) - W*H)*P1pred;
    else
        x1Est(i+1) = x1EstPred;
        P1 = P1pred;
    end
    
    % Store the estimated covariance of the estimation error
    P1Store(i+1) = P1;
    P1PredStore(i+1) = P1pred;


    %% Vehicle 2
    
    % Prediction
    x2EstPred = a*x2Est(i) + b*u2_bar(i);
    P2pred = a*P2*a' + b*sigma_u2^2*b';
    
    % Update
    if (rand(1) <= ProbGPS2)
        H = 1;
        InnCov = H*P2pred*H' + sigma_gps2^2;
        W = P2pred*H'*inv(InnCov);
        x2Est(i+1) = x2EstPred + W*(x2GPS(i+1) - H*x2EstPred);
        P2 = (eye(length(x2)) - W*H)*P2pred;
    else
        x2Est(i+1) = x2EstPred;
        P2 = P2pred;
    end
    
    % Store the estimated covariance of the estimation error
    P2Store(i+1) = P2;
    P2PredStore(i+1) = P2pred;
    
    %% Vehicle 1p2
    
    % Prediction
    x1p2EstPred = a*x1p2Est(i) + b*u1_bar(i);
    P1p2pred = a*P1p2*a' + b*sigma_u1^2*b';
    
    % Update
    pRadar = rand(1);
    if (pGPS1 <= ProbGPS1)
        if (pRadar <= ProbRadar)
            H = [1; -1];
            RadarMeasurements = x2Store(i+1) - x1Store(i+1) + randn(1)*sigma_radar + mu_radar;
            z = [x1GPS(i+1); RadarMeasurements - x2Est(i+1)];
            R = [sigma_gps1^2, 0; 0, sigma_radar^2 + P2];
        else
            H = 1;
            z = x1GPS(i+1);
            R = sigma_gps1^2;
        end
    else
         if (pRadar <= ProbRadar)
            H = -1;
            RadarMeasurements = x2Store(i+1) - x1Store(i+1) + randn(1)*sigma_radar + mu_radar;
            z = RadarMeasurements - x2Est(i+1);
            R = sigma_radar^2 + P2;
         end
    end
       
    if (pGPS1 <= ProbGPS1) || (pRadar <= ProbRadar)
        InnCov = H*P1p2pred*H' + R;
        W = P1p2pred*H'*inv(InnCov);
        x1p2Est(i+1) = x1p2EstPred + W*(z - H*x1p2EstPred);
        P1p2 = (eye(length(x1)) - W*H)*P1p2pred;
    else
        x1p2Est(i+1) = x1p2EstPred;
        P1p2 = P1p2pred;
    end
    
    % Store the estimated covariance of the estimation error
    P1p2Store(i+1) = P1p2;
    P1p2PredStore(i+1) = P1p2pred;    
    
end

%% Plot

figure(1), clf, hold on;
plot(t, x1Store, 'b');
plot(t, x1Est, 'r');
plot(t, x2Store, 'k--');
plot(t, x2Est, 'g--');
plot(t, x1p2Est, 'm:', 'LineWidth', 2);
legend('x1', 'x1Est', 'x2', 'x2Est', 'x1p2Est');
xlabel('t [s]');
ylabel('[m]');

figure(2), clf, hold on;
plot(t, P1Store, 'b');
plot(t, P1PredStore, 'r');
plot(t, P2Store, 'k--');
plot(t, P2PredStore, 'g--');
plot(t, P1p2Store, 'c:', 'LineWidth', 2);
plot(t, P1p2PredStore, 'm:', 'LineWidth', 2);
legend('P1', 'P1 Predicted', 'P2', 'P2 Predicted', 'P1p2', 'P1p2 Predicted');
xlabel('t [s]');
ylabel('[m^2]');
set(gca, 'YScale', 'log');

Error = x1Store(10:end) - x1Est(10:end);
figure(3), clf, hold on;
histogram(Error);
disp('Sample variance Vehicle 1');
var(Error)
disp('Estimated variance Vehicle 1');
P1

figure(4), clf, hold on;
autocorr(u1_bar - u1)

figure(5), clf, hold on;
autocorr(x1Est)

Error = x2Store(10:end) - x2Est(10:end);
figure(6), clf, hold on;
histogram(Error);
disp('Sample variance Vehicle 2');
var(Error)
disp('Estimated variance Vehicle 2');
P2
