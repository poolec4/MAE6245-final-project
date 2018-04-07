% Scott Barnes & Chris Poole
% MAE 6245: Robotic Systems
% Final Project 
clc; clear; close all;
% Simulates quadcopter with inverted pendulum
% Define Variables
g = 9.8;
M_r = 100;
m_p = 1;
L = 0.5;
I_x = 10;
I_y = 10;
I_z = 10;
% Create Open Loop Model
[A, B, C, D] = olSys(g, M_r, m_p, L, I_x, I_y, I_z);
ol_Sys = ss(A, B, C, D);
% Validate Controllability & Observability
% is_controllable(A, B);
% is_observable(A, C);
% Create Control Law
G = optimalLQR(A, B, C, D);
% Creat Closed Loop System
Ac = A-B*G;
cl_Sys = ss(Ac, B, C, D);
% Simulate System
t = 0:1:100;
% u = [t; -t; ones(1, length(t)); -ones(1, length(t))];
u = ones(4, length(t));
% Simulate Closed Loop System
lsim(cl_Sys, u, t) % Closed Loop System Clearly Stable
% Plot Order: X_q, Y_q, Z_q, Y_p, Theta_p, Roll, Pitch, Yaw
% Simulate Open Loop System
figure;
lsim(ol_Sys, u, t) % Open Loop System Clearly Unstable