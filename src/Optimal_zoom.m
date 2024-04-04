% metti a posto !!! la parte dei GRAFICI
%% Parameters
% ========================== Solves for beta and solves the NL optimization problem =====================
% ========================== for finding the optimal values of z and k

load('Data/C.mat')      % load desired environment
env = C;                % rename selected environment
Ts = 5;                 % settling time for P before starting zoom
theta = 40;             % angle of view (1m^2 from 2m)
gamma = 12;             % trade-off parameter
eFoV = 0.05;            % percentage error of view
err = 0.05;             % measure loss percentage
sigma_P = 0.25;         % kalman tracking/prediction error stdDev

% Generate Camera
cam = Camera(env, theta, 1, eFoV);

%% Coverage Factor Analysis
% ********* TRIAL AND ERROR *****************
beta = 0.62;
u = @(k) (1-exp(-k/beta));

tx = 0:0.001:1;
ts = tinv((tx+1)/2,inf);
lb = logical(ts >= 2);
hb = logical(ts <= 3);
bound = logical(lb.*hb);
mse = norm(tx(bound)-u(ts(bound)));

figure(1)
plot(ts, tx)
hold on
plot(ts, u(ts))

% ******** OPTIMIZATION ***********
tx = 0:0.00001:1;               % Real Uncertainty
ts = tinv((tx+1)/2, inf);       % Real Coverage factors calculation using Standard Gaussian/Normal distribution

% Approximated Uncertainty (target tracking confidence wrt a given coverage factor k)
u = @(k, beta) (1-exp(-k/beta));

% Zone of interest (where k in between 2 and 3)
lb = logical(ts >= 2);
hb = logical(ts <= 3);
bound = logical(lb.*hb); % logical array which contains true for elements of ts that are between 2 and 3

% Cost Function (formula 36)
u0 = @(beta) norm(tx(bound) - u(ts(bound), beta));

% Solves for optimal beta
beta = fmincon(u0, 0.5, [], [], [], [], 0, 1);

figure(2)
plot(ts, tx, 'LineWidth', 1.5)
hold on
plot(ts, u(ts, beta), 'LineWidth', 1.5)
legend('$U(k)$','$\widetilde{U}(k)$','interpreter','Latex','FontSize', 14);
xlabel('$k$','interpreter','Latex','FontSize', 14);
xlim([0 3.5])
fill([2 3 3 2 2],[-0.1 -0.1 1.2 1.2 -0.1],'b','facealpha',.1,'LineStyle','none')
grid(gca,'minor')
grid on
ax = gca;
ax.YLim = [-0.1 1.15];
ax.XLim = [0 3.5];
ax.XLabel.FontSize = 15;
ax.YLabel.FontSize = 15;
ax.XLabel.Interpreter = 'latex';
ax.XLabel.String = '$k$';
ax.YLabel.Interpreter = 'latex';
ax.YLabel.String = '$U(k)$';
legend({'$U(k)=2\Phi(k)-1$','$U(k)=1-e^{-{k \over b}}$','Domain of $U(k)$'},'Location','best');
ax.Legend.Interpreter = 'latex';
ax.Legend.FontSize = 10;
ax.Legend.Box = 'on';

%% Trade-off Parameter Analysis

% Gamma Test
y = [];
t = 1:5:100;
for gam = t
    [z, k] = cam.optimalZoom(gam, sigma_P);
    x = [z;k];
    y = [y x];
end

%%
% Solve to get gamma for different sigma_P values
P1 = 0.13;
y1 = [];
t1 = 1:50;
for gamma = t1
    [z,k] = cam.optimalZoom(gamma,P1);
    x1 = [z;k];
    y1 = [y1 x1];
end

P2 = 0.19;
y2 = [];
t2 = 1:50;
for gamma = t2
    [z,k] = cam.optimalZoom(gamma,P2);
    x2 = [z;k];
    y2 = [y2 x2];
end

P3 = 0.25;
y3 = [];
t3 = 1:50;
for gamma = t3
    [z,k] = cam.optimalZoom(gamma,P3);
    x3 = [z;k];
    y3 = [y3 x3];
end

figure()
plot(t, y(1,:)) % plot of z*
hold on
plot(t, y(2,:)) % plot of k*
legend('$z^*[m]$','$k^*$','interpreter','Latex','FontSize', 14);
xlabel('$\gamma$','interpreter','Latex','FontSize', 14);
title('$\gamma-test$','interpreter','Latex','FontSize', 14);

% sigma_P Test
y = [];
t = 0.05:0.005:0.55;
for sigma_p = t
    [z, k] = cam.optimalZoom(gamma, sigma_p);
    x = [z;k];
    y = [y x];
end

figure()
plot(t, y(1,:)) % plot of z*
hold on
plot(t, y(2,:)) % plot of k*
yline(cam.zmin,'--','LineWidth',1.5,'Color',[0, 0.4470, 0.7410]);
yline(cam.zMax,'--','LineWidth',1.5,'Color',[0, 0.4470, 0.7410]);
% grid(gca,'minor')
% grid on
% ax = gca;
% %x.YLim = [0 3.2];
% ax.XLim = [1 100];
% ax.XLabel.FontSize = 15;
% ax.YLabel.FontSize = 15;
% ax.XLabel.Interpreter = 'latex';
% ax.XLabel.String = '$\gamma$';
% ax.Legend.Interpreter = 'latex';
% ax.Legend.FontSize = 10;
% ax.Legend.Box = 'on';
legend('$z$','$k$','Limits','interpreter','Latex','FontSize', 14,'Location','best');
xlabel('$\sigma_p [m]$','interpreter','Latex','FontSize', 14);
title('$\sigma_p-test$','interpreter','Latex','FontSize', 14);

%% GRAFICI FINALI
tDown= 0.05:0.005:0.085;
kDown= (y(1,1:8)*tan(cam.Theta*pi/180))./tDown;
tUp = 0.50:0.01:0.55;
kUp= (cam.zMax*tan(cam.Theta*pi/180))./tUp;

figure()
subplot(2,1,1)
plot(t,y(1,:),'LineWidth',1)
title('io')
hold on;

yline(cam.zmin,'--','LineWidth',.8,'Color',[0, 0.4470, 0.7410]);
yline(cam.zMax,'--','LineWidth',.8,'Color',[0, 0.4470, 0.7410]);
xline(0.5,'--','LineWidth',.8,'Color','k');
xline(0.085,'--','LineWidth',.8,'Color','k');
grid on
ax = gca;
ax.YLim = [0.1 1.3];
ax.XLim = [0.05 0.55];
ax.XLabel.FontSize = 15;
ax.YLabel.FontSize = 15;
ax.YLabel.Interpreter = 'latex';
ax.YLabel.String = '$z^*$';


subplot(2,1,2)
plot(t,y(2,:),'LineWidth',1,'color',[0.8500, 0.3250, 0.0980])
hold on;
plot(tDown,kDown,'LineWidth',1,'color',[0.9290, 0.6940, 0.1250])
plot(tUp,kUp,'LineWidth',1,'color',[0.9290, 0.6940, 0.1250])
yline(2,'--','LineWidth',.8,'Color',[0.8500, 0.3250, 0.0980]);
yline(3,'--','LineWidth',.8,'Color',[0.8500, 0.3250, 0.0980]);
xline(0.5,'--','LineWidth',.8,'Color','k');
xline(0.085,'--','LineWidth',.8,'Color','k');
legend('1','2','3')
grid on
ax = gca;
%ax.YLim = [1.75 3.25];
ax.XLim = [0.05 0.55];
ax.XLabel.FontSize = 15;
ax.YLabel.FontSize = 15;
ax.YLabel.Interpreter = 'latex';
ax.YLabel.String = '$k^*$';
ax.XLabel.Interpreter = 'latex';
ax.XLabel.String = '$\sigma_p$';


% grafico sweep
figure()
subplot(2,1,1)
plot(t1,y1(1,:),'LineWidth',1)
hold on;
plot(t2,y2(1,:),'LineWidth',1)
plot(t3,y3(1,:),'LineWidth',1)
yline(cam.zmin,'--','LineWidth',1,'Color','k');
yline(cam.zMax,'--','LineWidth',1,'Color','k');
%grid(gca,'minor')
grid on
ax = gca;
ax.YLim = [0.1 1.3];
ax.XLim = [1 50];
ax.XLabel.FontSize = 18;
ax.YLabel.FontSize = 18;
ax.YLabel.Interpreter = 'latex';
ax.YLabel.String = '$z^\star$';
legend({'$\sigma_p = 0.13$','$\sigma_p=0.19$','$\sigma_p=0.25$'},'Location','best');
ax.Legend.Interpreter = 'latex';
ax.Legend.FontSize = 10;
ax.Legend.Box = 'on';

% grafico sweep
subplot(2,1,2)
plot(t1,y1(2,:),'LineWidth',1)
hold on;
plot(t2,y2(2,:),'LineWidth',1)
plot(t3,y3(2,:),'LineWidth',1)

%grid(gca,'minor')
grid on
ax = gca;
ax.YLim = [1.75 3.25];
ax.XLim = [1 50];
ax.XLabel.FontSize = 18;
ax.YLabel.FontSize = 18;
ax.YLabel.Interpreter = 'latex';
ax.YLabel.String = '$k^\star$';
ax.XLabel.Interpreter = 'latex';
ax.XLabel.String = '$\gamma$';

legend({'$\sigma_p = 0.13$','$\sigma_p=0.19$','$\sigma_p=0.25$'},'Location','best');
ax.Legend.Interpreter = 'latex';
ax.Legend.FontSize = 10;
ax.Legend.Box = 'on';


% VEDI SE TENERLI
figure()
yyaxis left
plot(t,y(1,:),'LineWidth',1.5)
yline(cam.zmin,'--','LineWidth',1.5,'Color',[0, 0.4470, 0.7410]);
yline(cam.zMax,'--','LineWidth',1.5,'Color',[0, 0.4470, 0.7410]);
grid(gca,'minor')
grid on
ax = gca;
ax.XLim = [0 1];
ax.YLim = [0 2.2];
ax.XLabel.FontSize = 15;
ax.YLabel.FontSize = 15;
ax.XLabel.Interpreter = 'latex';
ax.XLabel.String = '$\gamma$';
ax.YLabel.Interpreter = 'latex';
ax.YLabel.String = '$I(z)$';

yyaxis right
plot(t,y(2,:),'LineWidth',1.5)
hold on
yline(2,'--','LineWidth',1.5,'Color',[0.8500, 0.3250, 0.0980]);
yline(3,'--','LineWidth',1.5,'Color',[0.8500, 0.3250, 0.0980]);
grid(gca,'minor')
grid on
ax = gca;
ax.XLim = [0 1];
ax.YLim = [0 3.2];
ax.XLabel.FontSize = 15;
ax.YLabel.FontSize = 15;
ax.YLabel.Interpreter = 'latex';
ax.YLabel.String = '$u(k)$';


legend({'$u(k)=2\Phi(k)-1$','$u(k)=1-e^{-{k \over b}}$'},'Location','southeast');
ax.Legend.Interpreter = 'latex';
ax.Legend.FontSize = 10;
ax.Legend.Box = 'on';