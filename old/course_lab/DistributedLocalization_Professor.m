clear all;
close all;
clc;

%% let's try to set up a distributed localization or cooperative localization algorithm
% GPS gives us an absolute measurements
% velocity are relative measurements and RADAR can only measure the
% relative distance between vehicles

% so we have the model of an additional sensing system added into our
% distributed components

% remember that the noise that enters into z (z12) in KF should be White
% add the relative measurement z12 into the KF of veh 1

% as we will see addding another info makes sense : we reduce the
% uncertainty we have wrt to our estimate: by reducing sigma_radar we are
% gonna reach the uncertainty of vehicle 2

% also if we share info between the 2 vehicles we can add a a radar also to
% veh 2 in order to increase also the estimate on veh 2 as well (that's
% what we expect): if we do so we end up with a inconsistent filter:
% once veh 1 measures its abs position combining together the radar +
% position of x2 which x2 - x1, the covariance of the product of estimate of x1 times estimate of x2 is differnet than 0 
% so they are no more uncorrelated the 2 estimates since x1 uses the info
% from x2.


% if x1 keeps using the measurements of the radar to x2 it's fine
% but if x2 starts using the x1 measurements using the radar in turn we are
% injecting into KF of veh 2 a measurement correlated to the estimate of x2
% so this violate the KF a lot hypothesis.

% so the 2 cars have will have an estimate coeherent with the measurements
% but their location has a drift wrt actual location this becasue we are
% using correlated measurements (beaware of the cross-correlation between
% the estimates of uncertainties of the locations of the 2 vehicles)

% P1 and P2 in difference converge to 0 so we believe we have the correct
% estimate but that's not the case (inconsistent filter: info that comes
% are not consistent with the actual estimates)

% so when we start sharing info about ur state we are start correlating the
% estimates/measurements in the network:
% so when you share info among different components / own estimate with
% others vehicles, we need to know that we are correlating info with the
% others.
%

% So if we try to use their info for our estimate we have to discount this
% correlation because x2 - x1 is not totally new as info but is something
% that we already know. We can do it if we implement the estimtae of these 2
% vehicles using a single KF.

% so vehicle 2 sends to veh 1 its own estimate x2_hat and P2; also veh 2
% uses the info of veh 1 so this latter send x1_hat and P1 but also P12 to
% avoid the correlation problem and so also veh 2 will send P21 to veh 1.

%% Vehicle - 1 (very simple model)

Dt = 0.1;
t = 0:Dt:100;

x1 = rand(1); % random number between  0 and 1

u1 = 1 + sin(t); % sequence of inputs

% Friction effect
a = -0.1;
b = Dt*1.2; % velocity so should be integrated that's why the Dt

%% Vehicle - 2 (very simple model)
x2 = 2 + rand(1); % random number between  0 and 1: vehicle 2 starts in front of vehicle 1
u2 = 1 + cos(t); % sequence of inputs

%% System dynamics

x1Store = size(1, length(t));
x1Store(1) = x1;

x2Store = size(1, length(t));
x2Store(1) = x2;

for i=1:length(t)-1
    % Vehicle 1
    x1Store(i+1) = a * x1Store(i) + b * u1(i); %first order dynamics
    % Vehicle 2
    x2Store(i+1) = a * x2Store(i) + b * u2(i);
end

%% KF tracker

% Uncertainty in determining the actual velocity

% - actual measured input (sequence of velocity measurements collected about our
% system)

% Vehicle 1
mu_u1 = 0; % Calibrated Unbiased sensor
sigma_u1 = 0.1; % STD
u1_bar = u1 + randn(1, length(u1))*sigma_u1 + mu_u1;

% Vehicle 2
mu_u2 = 0; % Calibrated Unbiased sensor
sigma_u2 = 0.05; % STD
u2_bar = u2 + randn(1, length(u2))*sigma_u2 + mu_u2;

% - actual measured GPS (GPS gives us the sequence of locations of our
% vehicle over the road) = sequence of position measurements collected using GPS

% Vehicle 1
mu_GPS1 = 0;
sigma_GPS1 = 0.3;
x1GPS = x1Store + randn(1, length(x1Store))*sigma_GPS1 + mu_GPS1; % actual vehicle location + uncertainty
probGPS1 = 0.05; % probability of having a GPS reading (variability in our sensor z) 

% Vehicle 2
mu_GPS2 = 0;
sigma_GPS2 = 0.2;
x2GPS = x2Store + randn(1, length(x2Store))*sigma_GPS2 + mu_GPS2; % actual vehicle location + uncertainty
probGPS2 = 0.90; % probability of having a GPS reading (variability in our sensor z) = vehicle 2 more
% performant wrt vehicle 1 because we have more probability of using also z
% and also uncertainty on GPS and on u2 are smaller

% Relative RADAR measurements
mu_radar = 0;
sigma_radar = 0.1;
probRadar = 0.8;

% this are all info we have about our estimator (apart from the model
% knowledge)


%% Kalman Filters
% Vehicle 1
x1Est = zeros(1, length(t)); % is zero since we do not have any info
P1 = 1e2; % variance = knowledge about the initial location of the vehicle
P1Store = zeros(1, length(t));
P1PredStore  = zeros(1, length(t));

% We assume to have GPS reading at position x(0) at time 0 or not ? 
% if we have the first GPS reading at time 1 not 0 which means we start at
% t=0 (very beginning) with a prediction and then update otherwise we have
% to make an update and prediction

% Vehicle 2
x2Est = zeros(1, length(t)); % is zero since we do not have any info
P2 = 1e2; % variance = knowledge about the initial location of the vehicle
P2Store = zeros(1, length(t));
P2PredStore  = zeros(1, length(t));

% Vehicle 1 plus 2 as additional 'sensor'/measurement of vehicle 2 plugged into vehicle 1 (1p2)
x1p2Est = zeros(1, length(t)); % is zero since we do not have any info
P1p2 = 1e2; % variance = knowledge about the initial location of the vehicle
P1p2Store = zeros(1, length(t));
P1p2PredStore  = zeros(1, length(t));

for i=1:length(t)-1
    
    %% Vehicle 1 (estimates its own position making a measurment with the RADAR and ask to veh 2 its absolute reference (x2) so that veh 1 can have an additional measurement so
    % this should decrease the uncertainty of x1 but veh 1 has to ask to
    % veh 2 'what is your position' and veh 2 has to reply
    % OFC veh 2 does not know it own position but only the estimate of its
    % own position

    % Prediction (use knowledge of model and sensor readings)
    x1EstPred = a*x1Est(i) + b*u1_bar(i); % knowledge = what GPS should read if the position estimated is correct
    P1pred = a*P1*a' + b*sigma_u1^2*b'; % covariance matrix of the noise affecting the model

    % Update
    
    % to simulate if we have a measurement/reading or not with a certain
    % prob
    pGPS1 = rand(1); % will be used twice since we will test vehicle 1 with radar or not
    if(pGPS1 <= probGPS1)
        H = 1; % since we are measuring the position and our state is the position
        CovInn = H * P1pred * H' + sigma_GPS1^2; % uncertainty on the z
        W = P1pred * H' * inv(CovInn);
        x1Est(i+1) = x1EstPred + W*(x1GPS(i+1) - H*x1EstPred);
        P1 = (eye(length(x1)) - W*H)*P1pred;
    else % no measurement ==> no update
        x1Est(i+1) = x1EstPred;
        P1 = P1pred; % estimated cov of error = cov of predicted
    end

    % Store the estimated covariance of the estimation error = P1
    % (precision of our filter)
    P1Store(i+1) = P1;
    P1PredStore(i+1) = P1pred;
    
    %% Vehicle 2 (transmits to veh 1 its own estimate so its value of x2 since veh 1 does not know it)
    % Prediction (use knowledge of model and sensor readings)
    x2EstPred = a*x2Est(i) + b*u2_bar(i); % knowledge = what GPS should read if the position estimated is correct
    P2pred = a*P2*a' + b*sigma_u2^2*b'; % covariance matrix of the noise affecting the model

    % Update
    
    % to simulate if we have a measurement/reading or not with a certain
    % prob
    if(rand(1) <= probGPS2)

        H = 1; % since we are measuring the position and our state is the position
        CovInn = H * P2pred * H' + sigma_GPS2^2; % uncertainty on the z
        W = P2pred * H' * inv(CovInn);
        x2Est(i+1) = x2EstPred + W*(x2GPS(i+1) - H*x2EstPred);
        P2 = (eye(length(x2)) - W*H)*P2pred;
    else % no measurement ==> no update
        x2Est(i+1) = x2EstPred;
        P2 = P2pred; % estimated cov of error = cov of predicted
    end

    % Store the estimated covariance of the estimation error = P2
    % (precision of our filter)
    P2Store(i+1) = P2; % P2 uncertainty we have on estimation error
    P2PredStore(i+1) = P2pred;

    %% Vehicle 1p2  (vehicle 1 plus the additional radar measurements): we are trying to see if there is a difference between only GPS or GPS + radar

    % Prediction (use knowledge of model and sensor readings)
    x1p2EstPred = a*x1p2Est(i) + b*u1_bar(i); % knowledge = what GPS should read if the position estimated is correct
    P1p2pred = a*P1p2*a' + b*sigma_u1^2*b'; % covariance matrix of the noise affecting the model

    % Update
    
    % to simulate if we have a measurement/reading or not with a certain
    % prob
    pRadar = rand(1);
    if(pGPS1 <= probGPS1)
        if(pRadar <= probRadar)
        % if here we have both GPS and radar
            H = [1; -1]; % 1 for the GPS (absolute reference) and -1 for the radar (relative reference)
            radarMeasurements = x2Store(i+1) - x1Store(i+1) + randn(1)*sigma_radar + mu_radar; % removed from radar z the x2Est
            z = [x1GPS(i+1); radarMeasurements - x2Est(i+1)];
            R = [sigma_GPS1^2, 0; 0, sigma_radar^2 + P2]; % COV(z) = [2x2] matrix = off-diagonal are 0 since GPS and radar measurements are uncorrelated
        else
            % GPS only
            H = 1; % 1 for the GPS (absolute reference) and -1 for the radar (relative reference)
            z = x1GPS(i+1); 
            R = sigma_GPS1^2;
        end
    else
        % only radar
        if(pRadar <= probRadar)
            H = -1; % 1 for the GPS (absolute reference) and -1 for the radar (relative reference)
            radarMeasurements = x2Store(i+1) - x1Store(i+1) + randn(1)*sigma_radar + mu_radar;
            z = radarMeasurements - x2Est(i+1);
            R = sigma_radar^2 + P2;
        end
    end

    if ((pGPS1 <= probGPS1) || (pRadar <= probRadar))
        % GPS or radar
        CovInn = H * P1p2pred * H' + R; % uncertainty on the z
        W = P1p2pred * H' * inv(CovInn);
        x1p2Est(i+1) = x1p2EstPred + W*(z - H*x1p2EstPred);
        P1p2 = (eye(length(x1)) - W*H)*P1p2pred;
    else % no measurement ==> no update => nor GPS nor the radar ==> go for the prediction
        x1p2Est(i+1) = x1p2EstPred;
        P1p2 = P1p2pred; % estimated cov of error = cov of predicted
    end

    % Store the estimated covariance of the estimation error = P2
    % (precision of our filter)
    P1p2Store(i+1) = P1p2; % P2 uncertainty we have on estimation error
    P1p2PredStore(i+1) = P1p2pred;

end

%% Plots

figure(1), clf, hold on;
plot(t, x1Store, 'b');
plot(t, x1Est, 'r');
plot(t, x2Store, 'k--');
plot(t, x2Est, 'g--');
plot(t, x1p2Est, 'm:', 'LineWidth', 3);
legend('x1 actual trajectory','x1 estimated trajectory', 'x2 actual trajectory','x2 estimated trajectory', 'x1p2 estimated trajectory'); % good tracking of the position
xlabel('t [s]');
ylabel('[m]');

figure(2), clf, hold on;
plot(t, P1Store, 'b'); % to have an idea of the uncertainty behaviour
plot(t, P1PredStore, 'r'); % to have an idea of the uncertainty behaviour
plot(t, P2Store, 'k--'); % to have an idea of the uncertainty behaviour
plot(t, P2PredStore, 'g--'); % to have an idea of the uncertainty behaviour
plot(t, P1p2Store, 'c:','LineWidth', 3); % to have an idea of the uncertainty behaviour
plot(t, P1p2PredStore, 'm:','LineWidth', 3); % to have an idea of the uncertainty behaviour
legend('P1','P1 predicted', 'P2','P2 predicted', 'P1p2','P1p2 predicted');
xlabel('t [s]');
ylabel('[m^2]');
set(gca, 'YScale','log'); % to have a clearer look
% P1 pred is a little bit higher since we are using prediction 
% so the uncertainty will increase

% increasing sigma_u1 = uncertainty on the model we will have P1pred quite
% high wrt actual


% we reached a plateau which is the min possible estimation error =
% tradeoff between uncertainty on GPS and the one on FW velocity

% plateau given by the combination of uncertainties of model and of z and
% since both of them are not 0 we are never gonna reach 0 unless the system
% is A.S


% if we have another vehicle:
% sharing info about estimate between 2 vehicles, our uncertainty will decrease


%% let's see what is the histogram distribution of the error by assuming an 
%% ergodic process
Error = x1Store(10:end) - x1Est(10:end); % idea of uncertainty on x: removed outliers due to ICs (first 10 estimates removed)
figure(3), clf, hold on;
histogram(Error); % shoudl be Gaussain RV since linear system and all uncertainties are Gaussian so uncertainty on vehicle position is GAUSSIAN
disp('Sample variance of vehicle 1');
% Sanity check
var(Error) % sample variance computed on data
disp('Estimated variance of vehicle 1');
P1 % estimated variance (filter precision) if equal to var(error) means our model behaves as expected so we can estimate the variance


% test 
% Uncertainty coming out from KF is Markovian Process while
% uncertainty going into the model of the sensor should be white
figure(4), clf, hold on;
autocorr(u1_bar - u1) % autocorr(normalized by noise variance) of the noise /uncertainty on velocity of x1
% when we consider the samples of uncertainties values at same time instants or different time
% instants: tells us what are the characteristic of the process generating eps_k from
% stochastic POV: the process is white as we can see since it has cov = 0
% for all k and k+i for all i and equal to sigma^2 when we compute the
% autocov so cov at same time ==> if white we expect a Kroncker delta in 0
% and 0 elsewhere so we have no relation between the time sequence of
% samples

figure(5), clf, hold on;
autocorr(x1Est) % uncertainty on estimate is is given by x1ESt

Error = x2Store(10:end) - x2Est(10:end); % idea of uncertainty on x: removed outliers due to ICs (first 10 estimates removed)
figure(6), clf, hold on;
histogram(Error); % shoudl be Gaussain RV since linear system and all uncertainties are Gaussian so uncertainty on vehicle position is GAUSSIAN
disp('Sample variance of vehicle 2');
% Sanity check
var(Error) % sample variance computed on data
disp('Estimated variance of vehicle 2');
P2 % estimated variance (filter precision) if equal to var(error) means our model behaves as expected so we can estimate the variance







