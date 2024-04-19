function UnicycleUKF()

%% Set-up

Dt = 0.1;
t = 0:Dt:10;    % [s]

% Unicycle state
s0 = [1; 1; pi/2];  % [x,y,theta]
sStore = zeros(length(s0), length(t));
sStore(:,1) = s0;

% Vehicle inputs
u = [sin(t/10)*2; 
    sin(t/5).*cos(t/4)*1.5];

% Inputs uncertainty
mu_u = zeros(size(u,1),1);
Q = rand(size(u,1),size(u,1))-0.5;
Q = Q*Q';

% UWB anchors
n_UWB = 10;
UWB = (rand(2,n_UWB)-0.5)*20;

% UWB uncertanties
mu_r = zeros(size(UWB,2),1);
R = diag(rand(size(UWB,2),1));


%% Simulation

sEst = zeros(length(s0), length(t));
P_UKF = (20/3)^2*eye(length(s0));

for cT=1:length(t)-1
    % Unicycle dynamic
    sStore(:,cT+1) = VehicleDynamic(sStore(:,cT), u(:,cT), Dt);
    % Measure the robot inputs
    um = InputMeasurements(u(:,cT), mu_u, Q);
    % Ranging data
    z = Ranging(sStore(:,cT+1), UWB);
    % Ranging measurements
    zm = RangingMeasurements(z, mu_r, R);
    
    %% UKF
    
    % Prediction step
    SigmaPointsPred = SigmaPointGeneration(sEst(:,cT), P_UKF);
    for i=1:size(SigmaPointsPred,2)
        SigmaPointsPred(:,i) = VehicleDynamic(SigmaPointsPred(:,i), um, Dt);
    end
    
end


%% Plot
PlotData(sStore, UWB);



function s_new = VehicleDynamic(s, u, Dt)

% Inputs
x = s(1);
y = s(2);
theta = s(3);
v = u(1);
omega = u(2);

% Dynamics
x_new = x + cos(theta)*v*Dt;
y_new = y + sin(theta)*v*Dt;
theta_new = theta + omega*Dt;

% Output
s_new = [x_new; y_new; theta_new];


function um = InputMeasurements(u, mu, Q)

epsilon = mvnrnd(mu, Q);
um = u + epsilon;



function z = Ranging(s, UWB)

% Number of UWB
n_UWB = size(UWB,2);

z = zeros(n_UWB, 1);
for i=1:n_UWB
    z(i) = sqrt(sum((s(1:2) - UWB(:,i)).^2));
end



function zm = RangingMeasurements(z, mu, R)

nu = mvnrnd(mu, R);
zm = z + nu;



function SigmaPoints = SigmaPointGeneration(s, P)

% State dimension
n = length(s);

% Cholesky decomposition
A = chol(P);

SigmaPoints = zeros(n, 2*n);
for i=1:n
    SigmaPoints(:,i) = s + sqrt(n)*A(i,:)';
    SigmaPoints(:,i+n) = s - sqrt(n)*A(i,:)';
end




function PlotData(sStore, UWB)

% Number of UWB
n_UWB = size(UWB,2);

figure(1), clf, hold on;
plot(sStore(1,:), sStore(2,:), 'b');
for i=1:n_UWB
    plot(UWB(1,i), UWB(2,i), 'ks', 'LineWidth', 2, 'MarkerSize', 10);
end
legend('Actual robot', 'UWB', 'Location', 'best');
xlabel('x [m]'); ylabel('y [m]');








