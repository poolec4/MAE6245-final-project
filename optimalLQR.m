% Scott Barnes & Chris Poole
% MAE 6245: Robotic Systems
% Final Project 

% Creates open loop state-space system for a quadrotor with attached
% inverted pendulum

%   Generates Optimal LQR Controller. 
% Inputs
%   A: A matrix of Open Loop System
%   B: B matrix of Open Loop System
%   C: C matrix of Open Loop System
%   D: D matrix of Open Loop System
% Outputs
%   G: Optimal LQR Controller

function G = optimalLQR(A, B, C, D)
    Q = C'*C; % Weight Matrix: Equally Weighs Observed Variables. 
    % v = ones(1, 16); 
    % Q = diag(v); % Equally weighs all variables
    Q(14, 14) = 1; % Pendulum Angle Weight
    Q(1, 1) = 10; % X Position Weight
    Q(2, 2) = 10; % Y Position Weight
    Q(3, 3) = 10; % Z Position Weight
    v = ones(size(B, 2), 1);
    R = diag(v);
    H = [A -B*inv(R)*B'; -Q -A']; % Hamiltonain Matrix
    [V, E] = eig(H); % Eigenavlues of Hamiltonian
    ind = 0;
    for i = 1:size(E, 1) % Find stable eigenvalues
        if real(E(i, i)) < 0
            ind = ind + 1;
            T(:, ind) = V(:, i);
        end
    end
    T1 = T(1:size(T,1)/2, :);
    T2 = T(size(T,1)/2+1:size(T,1), :);
    % M = T2*inv(T1);
    M = T2*pinv(T1); % Create M Matrix
    G = real(inv(R)*B'*M); % Calculate Control Law
    % K = lqr(A, B, Q, R) % Uncomment to verify with MATLAB's LQR optimizer
end