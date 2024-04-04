clear
close all
clc

% ======================================================================================================
%  Simulating a tracking system where a camera is tracking a moving object
%  within a defined environment. The object's movement is modeled as a random walk, and the camera 
%  uses a Kalman filter to predict the object's future position. 
%  The script also calculates the tracking error, which is the difference between the object's actual
%  position and its predicted position
% ======================================================================================================

%% Environment

N = 500;                                            % # of simulation iterations

B = [0 0; 0 9; 7 9; 7 6; 3 6; 3 3; 7 3; 7 0];       % C Environment
% B = [0 0; 0 7; 4 7; 4 3; 6 3; 6 0];               % L Environment

res = 1; % cellularization of the environment

env = Environment(B,{});
env.cellularize(res);

%% Generate Camera
Ts = 5;             % settling time for P before starting zoom
theta = 40;         % angle (1m^2 from 2m)
gamma = 12;         % trade-off parameter
eFoV = 0.15;        % percentage error of view
err = 0.00;         % measure loss percentage

cam = Camera(env, theta, 10, eFoV);

%% Starting Point of Real Target/Object Trajcetory
j = 1;
while j <= N
    pos = randi([min(min(B)), max(max(B))], 1, 2); % random pos withing env respecting B boundaries points
    bar = [(min(B(:,1)) + max(B(:,1)))/2, (min(B(:,2)) + max(B(:,2)))/2]; % center of env by averaging min and max coords (x, y) of B
    
    % verify if starting point is withing polygon/environment borders and not on them
    [~,on] = inpolygon(pos(1), pos(2), B(:,1), B(:,2));
    if ~on % on the edge since on means on polygon edge => on = true
        % loops until a random position is not on the edge of the env so is within env
        while ~on 
            pos = randi([min(min(B)), max(max(B))], 1, 2);
            [~,on] = inpolygon(pos(1), pos(2), B(:,1), B(:,2));
        end
    end
    vin = bar-pos; % target initial velocity directed towards env center (bar)
    
    % checks if a small step in the direction of the velocity would keep the object within env
    in = inpolygon(pos(1) + vin(1)/100, pos(2) + vin(2)/100, B(:,1), B(:,2));
    if ~in % target outside env
        vin = -vin; % ensuring convering towards env center bar
    end

    % Real Trajectory model of the target (as random walk) 
    a = 1;                      % speed [m/s]
    Tc = 0.1;                   % sampling period [s]
    v = a*(vin)/norm(vin);      % speed vector [vx,vy] normalized
    x = [pos, v];               % state vector
    sigma_q = 0.15;             % process noise variance

    % process noise covariance matrix
    Qc = sigma_q*[0 0 0 0; 0 0 0 0; 0 0 1 0; 0 0 0 1]; % random speed (q = 0.x)
    % random acceleration (q = x)
    % Qc = q*[Tc^4/4, 0, Tc^3/2, 0; 0, Tc^4/4, 0, Tc^3/2; Tc^3/2, 0, Tc^2, 0; 0, Tc^3/2, 0, Tc^2]; % equation 27

    % Transition matrix  (generates the next trajectory point)
    Ac = [1, 0, Tc, 0; 0, 1, 0, Tc; 0, 0, 1, 0; 0, 0, 0, 1]; 

    % Filter target trajectory model
    T = 0.3;             % sampling period of Kalman filter
    sigma_q = 10;        % process noise variance

    % process noise covariance matrix (random acceleration)
    Q = sigma_q*[T^4/4, 0, T^3/2, 0; 0, T^4/4, 0, T^3/2; T^3/2, 0, T^2, 0; 0, T^3/2, 0, T^2];
    
    % Transition matrix  
    A = [1, 0, T, 0; 0, 1, 0, T; 0, 0, 1, 0; 0, 0, 0, 1]; % equation 26

    % Filter Measurement model
    H = [1 0 0 0; 0 1 0 0];     % measurement matrix
    r = cam.FoV*cam.eFoV;       % measurement noise standard deviation
    R = r^2*eye(2);             % measurement noise covariance matrix

    % One Step Ahead Prediction
    s = round(T/Tc); % round to nearest int (i.e. this case is 3)
    out = false;     % inside a polygon

    % Filter Initialization 
    % NOTE it must be set in the point where the first detection has been done
    y = (H*x(1,:)' + sqrt(R)*randn(2,1))'; % gaussian noise added scaled by sqrt(R)

    x_pred = [y(1,:),0 0];
    x_pred(1+s,:) = [y(1,:),0 0];          % state at next time step s+1 based on current measurement and assuming 0 vel and acceleration
    P_pred = blkdiag(R, eye(2));
    P_pred(:,:,1+s) = blkdiag(R, eye(2));  % state covariance at next time step s+1

    stdDev = sqrt(P_pred(:, :, 1));
    stdDevMax = max((stdDev(1,1) + stdDev(2,2))/2 , T*(stdDev(3,3) + stdDev(4,4))/2);
    stdDevMax(1+s) = max((stdDev(1,1) + stdDev(2,2))/2 , T*(stdDev(3,3) + stdDev(4,4))/2); % max stdDev for the next time step s+1

    zoom = [cam.zMax; cam.FoV/stdDevMax(1)];
    zoom(:,1+s) = [cam.zMax; cam.FoV/stdDevMax(1+s)]; % zoom level for the next time step s+1 based on the camera's maximum zoom level and ratio of camera's FoV to max stdDev

    % the lines with 1+s are preparing for next iteration
    % where the predicted states and covariances will be updated based on new measurements
    
    for i = 2:s
        % New target Trajectory point
        x = [x; (Ac*x(i-1,:)' + Qc*randn(4,1))']; % Qc for modeling a random speed
        % verify if new trajectory point is inside our env
        in = inpolygon(x(i,1),x(i,2),B(:,1),B(:,2));
        if ~in
            out = true; % outside a polygon
        end
    end

    count = s;
    if ~out % inside a polygon
        i = s+1;
    else % outside a polygon
        i=0;
    end
    while i ~= 0
        % New target Trajectory point
        x = [x; (Ac*x(i-1,:)' + Qc*randn(4,1))'];
        
        if count >= s
            count = 1;
            % Detection
            if randi([0,100])/100 > err
                r = cam.FoV*cam.eFoV;                   
                R = r^2*eye(2);                           
                y(i,:) = (H*x(i,:)' + sqrt(R)*randn(2,1))';
            else
                y(i,:) = [0 0];  % loss of a measurement
            end
            % Prediction
            [x_pred(i+s,:), P_pred(:,:,i+s)] = kalman(A, H, Q, R, y(i,:), x_pred(i,:), P_pred(:,:,i));
            stdDev = sqrt(P_pred(:,:,i+s));
            stdDevMax(i+s) = max((stdDev(1,1) + stdDev(2,2))/2 , T*(stdDev(3,3) + stdDev(4,4))/2);
            % Zoom Control
            if i >= Ts*s
                [z,k] = cam.optimalZoom(gamma, stdDevMax(i+s)); % optimal zoom level computation
                zoom(:,i+s) = [z;k];
                cam.updateZ(zoom(1, i+s)); % camera zoom level update
            else
                zoom(:,i+s) = [cam.zMax; cam.FoV/stdDevMax(i+s)];
            end
        else
            count = count + 1;
        end
        
        % verify if new trajectory point is inside our env
        in = inpolygon(x(i,1), x(i,2), B(:,1), B(:,2));
        if ~in
            i=0;
        else
            i = i+1;
        end
    end

    % Correct Indices
    for t1 = 1:s:size(y,1)
        % first 2 lines replaces the predicted states and covariances from t1+1 to t1+s-1 with the predicted state / covariance at t1+s. 
        % The multiplication with ones(s-1,4) / ones(4,4,s-1) is to replicate the row vector x_pred(t1+s,:)/matrix P_pred(:,:,t1+s) into a matrix with s-1 rows / 3D array with s-1 matrices.
        x_pred(t1+1:t1+s-1,:) = x_pred(t1+s,:).*ones(s-1,4);
        P_pred(:,:,t1+1:t1+s-1) = P_pred(:,:,t1+s).*ones(4,4,s-1);
        y(t1+1:t1+s-1,:) = y(t1,:).*ones(s-1,2); % analogous
    end

    % IMPORTANT MARK !!!
    % P_pred is a 3D matrix that stores the predicted covariance matrices for each time step
    % in the simulation. Each slice of this 3D matrix (i.e., P_pred(:,:,i)) is a 4x4 covariance matrix 
    % for the state at time step i.


    % Tracking performance index
    if size(x,1)>Ts*s
        pred = x_pred((Ts*s):s:size(x,1),1:2); % predicted positions
        real = x((Ts*s):s:size(x,1),1:2);      % real positions
        track = sqrt((real(:,1)-pred(:,1)).^2 + (real(:,2)-pred(:,2)).^2); % tracking error as Euclidean distance between the real and predicted positions
        track_err(j)    = mean(track);
        track_errMax(j) = max(track);
        j=j+1;
    end 
end

fprintf('Mean Tracking Error: %2.3f \n', mean(track_err));
fprintf('Max Tracking Error:  %2.3f \n', mean(track_errMax));
fprintf('Max of Max Tracking Error: %2.3f \n', max(track_errMax));

%% Plot (prima era commentato tutto da qua fino alla fine)
plots = true;
if plots 
    figure()
    % tracking plots
    for t = 1:size(x,1)
        env.plotBorders()
        hold on
        scatter(x(t,1),x(t,2),30,[0.8500, 0.3250, 0.0980],'filled')
        plot(x(1:t,1),x(1:t,2),'LineWidth',1.5,'Color',[0.8500, 0.3250, 0.0980])
        plot(y(1:t,1),y(1:t,2),'o','MarkerSize',3,'Color',[0, 0.4470, 0.7410]) % measurements
        plot(x_pred(1:t,1),x_pred(1:t,2),'LineWidth',1.5,'Color',[0.9290, 0.6940, 0.1250])
        % Plot Camera's position, zoom level and its FoV
        cam.X(1) = x_pred(t,1);
        cam.X(2) = x_pred(t,2);
        cam.updateZ(zoom(1,t)); % zoom(1, t) = camera zoom level at the time step t
        cam.plot()
        hold off
        axis equal
        pause(Tc)
    end

    % % Uncertainty
    beta = 0.6161;
    u = @(k) (1-exp(-k/beta));
    % Information Lost (due to distance)
    I = @(z) -(exp(z-cam.zmin)-1);

    figure()
    subplot(3,1,1)
    plot(zoom(1,1:s:end))
    xlabel('t[s]')
    ylabel('z[m]')
    title('Zoom level over time')
    ylim([0,cam.zMax+0.1])
    subplot(3,1,2)
    plot(abs(I(zoom(1,1:s:end))))
    title('Information loss as function of zoom level')
    subplot(3,1,3)
    plot(u(zoom(2,1:s:end)))
    title('Uncertainty as function of zoom level')

    % Zoom Analysis
    figure()
    for t = 1:s:size(x,1)
        % Plot Camera
        cam.X(1) = x_pred(t,1);
        cam.X(2) = x_pred(t,2);
        cam.updateZ(zoom(1,t));
        scatter(0,0,30,[0.9290, 0.6940, 0.1250],'filled')
        hold on
        rectangle('Position',[-cam.FoV -cam.FoV 2*cam.FoV 2*cam.FoV])
        scatter(x(t,1)-cam.X(1),x(t,2)-cam.X(2),30,[0.8500, 0.3250, 0.0980],'filled')
        %  three circles representing 1, 2, and 3 standard deviations of the maximum standard deviation
        rectangle('Position',[-stdDevMax(t) -stdDevMax(t) 2*stdDevMax(t) 2*stdDevMax(t)],'Curvature',1)
        rectangle('Position',[-3*stdDevMax(t) -3*stdDevMax(t) 2*3*stdDevMax(t) 2*3*stdDevMax(t)],'Curvature',1)
        rectangle('Position',[-2*stdDevMax(t) -2*stdDevMax(t) 2*2*stdDevMax(t) 2*2*stdDevMax(t)],'Curvature',1)
        hold off
        axis equal
        xlim([-env.Res-0.1 env.Res+0.1])    
        ylim([-env.Res-0.1 env.Res+0.1])
        pause()
    end
end