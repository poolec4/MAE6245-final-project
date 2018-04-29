% Scott Barnes & Chris Poole
% MAE 6245: Robotic Systems
% Final Project 

% Test script to view plotting of quadrotor

close all
clear all
clc

addpath('common_functions')

x_g = [0,0,0]; % global position [x,y,z]
eangles = [deg2rad(0),deg2rad(0),deg2rad(0)]; % roll (gamma), pitch (beta), yaw (alpha) in degrees

figure(1)
view([55 30.0])
draw_quadrotor(x_g,eangles)
axis([-0.5 0.5 -0.5 0.5 -0.5 0.5])
while 1
    for z = 0:0.05:1
        x_g = [0,0,z];
        eangles = [0,0,(z*2*pi)];
        draw_quadrotor(x_g,eangles)
        axis([-1 1 -1 1 -1 1])
        drawnow
    end
    
    for z = 1:-0.1:0
        x_g = [0,0,z];
        eangles = [0,(z*pi/2)-pi/2,2*pi];
        draw_quadrotor(x_g,eangles)
        axis([-1 1 -1 1 -1 1])
        drawnow
    end
    
    for z = 1:-0.1:0
        x_g = [0,0,0];
        eangles = [(z*pi)-pi,-pi/2,2*pi];
        draw_quadrotor(x_g,eangles)
        axis([-1 1 -1 1 -1 1])
        drawnow
    end
    
    for z = 0:0.1:1
        x_g = [0,0,0];
        eangles = [-pi,z*pi/2-pi/2,2*pi];
        draw_quadrotor(x_g,eangles)
        axis([-1 1 -1 1 -1 1])
        drawnow
    end
    
    for z = 0:0.1:1
        x_g = [0,0,0];
        eangles = [-pi,0,2*pi];
        draw_quadrotor(x_g,eangles)
        axis([-1 1 -1 1 -1 1])
        drawnow
    end
end