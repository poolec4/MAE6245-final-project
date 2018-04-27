%% optimal_lqr
%   Generates Optimal LQR Controller. 
% Authorship:
%   Scott Barnes
%   The George Washington University
%   MAE 6245: Robotic Systems
%   Final Project: Quadrotor Control
% Inputs
%   A: A matrix of Open Loop System
%   B: B matrix of Open Loop System
%   C: C matrix of Open Loop System
%   D: D matrix of Open Loop System
% Outputs
%   G: Optimal LQR Controller

function G = optimalLQR(A, B, C, D)
    % Q = C'*C; % Equally Weighs Observed Variables. Ignores those not in
    % observer
    v = ones(1, 16); 
    Q = diag(v); % Equally weighs all variables
    % Q(14, 14) = 0.001; % Pendulum Angle - Set for large displacement, small maximum angle
    % Q(13, 13) = 0.001;
    % Q(13, 13) = 1000; % Pendulum Displacement - Set for minimal displacement, larger angle
    Q(1, 1) = 100; % X Position
    Q(2, 2) = 100; % Y Position
    Q(3, 3) = 100; % Z Position
    v = ones(size(B, 2), 1);
    R = diag(v);
    % R = 1;
    H = [A -B*inv(R)*B'; -Q -A'];
    [V, E] = eig(H);
    ind = 0;
    for i = 1:size(E, 1)
        if real(E(i, i)) < 0
            ind = ind + 1;
            T(:, ind) = V(:, i);
        end
    end
    T1 = T(1:size(T,1)/2, :);
    T2 = T(size(T,1)/2+1:size(T,1), :);
    % M = T2*inv(T1);
    M = T2*pinv(T1);
    G = real(inv(R)*B'*M);
    % K = lqr(A, B, Q, R) % Uncomment to verify with MATLAB's LQR optimizer
end