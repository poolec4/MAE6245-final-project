% Scott Barnes & Chris Poole
% MAE 6245: Robotic Systems
% Final Project 

% Test script to view coordinate frame transformations

close all
clear all
clc

x_g = [0;0;3]; % global position [x,y,z]
eangles = [deg2rad(0),deg2rad(0),deg2rad(0)]; % roll (gamma), pitch (beta), yaw (alpha) in degrees

u = eangles(1);
v = eangles(2);
w = eangles(3);

R_quad = rot_z(w)*rot_y(v)*rot_x(u);
R_e_q = [0 1  0; 1 0 0; 0 0 -1];

clf(1)
figure(1)
hold on
grid on
axis equal
view([55 30.0])
plotCoordFrame3d(R_e_q*R_quad,x_g,1);
% axis([-1 1 -1 1 -1 1])