function [kp, ki, kd] = get_PID(sysP, wgc, phim, alpha)
    %   ============= PID controller design with Bode's method ==================
    %
    %   [KP, KI,KD] = GET_PID(SYSP, WGC, PHIM, ALPHA) designs a PID controller 
    %   with Bode's method for the plant SYSP, using WGC [rad/s] as gain 
    %   crossover frequency and PHIM [deg] as phase margin. ALPHA is the ratio 
    %   between the integral time TI=KP/KI and the derivative time TD=KD/KP, 
    %   i.e. ALPHA = TI/TD.
    %
    %   The controller tf is equal to (in the continuous-time case):
    %       C(s) = (KP + KI/s + KD*s) = KP*(1 + 1/(s*TI) + s*TD)
    %
    %   The design is performed either in continuous or discrete time,
    %   depending on the plant tf SYSP. The integrator tf in the discrete time
    %   case is considered as Ts*z/(z-1) (Backward Euler).
     
    %   The phase margin, phim, is a measure of the system's stability. 
    %   The gain crossover frequency, wgc, is the frequency at which the gain of the system is 1 (0 dB).
    %   These values are chosen based on the desired performance of the system.

    %   kp: The proportional gain is chosen such that the gain of the open-loop system is 1 (0 dB) 
    %   at the gain crossover frequency.

    %   Calculate the integral and derivative times: The integral time is chosen such that the phase
    %   of the open-loop system at the gain crossover frequency is equal to the desired phase margin.
    %   The derivative time is ti/alpha, where alpha is a chosen ratio between the integral and 
    %   derivative times.

    %   Calculate the integral and derivative gains: The integral gain is calculated as kp/ti,
    %   and the derivative gain is calculated as kp*td.
    % 
end