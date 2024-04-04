%% KALMAN Kalman Filter one step ahead predictor
%% ==================== DESCRIPTION =====================
% A = state-transition matrix
% H = measuremente matrix
% Q = covariance matrix of the process noise
% R = covariance matrix of the measurement noise
% y = new coming measurement from sensor (i.e camera)
% x = state of the target (posX, posY, velX, velY)
% P = prediction error covariance matrix
%% ================================================
function [x_pred, P_pred] = kalman(A, H, Q, R, y, x, P)
    % Propagation
    if (y(1)==0 && y(2)==0)  
        H = 0.*H;
    end 

    % kalman gain matrix
    K = A*P*H'*(H*P*H' + R)^(-1);
    % prediction state
    x_pred = (A*x' + K*(y' - H*x'))';
    % covariance matrix of prediction errors
    P_pred = A*P*A' + Q - A*P*H'*(H*P*H' + R)^(-1)*H*P*A';
    

end

