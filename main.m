% Scott Barnes & Chris Poole
% MAE 6245: Robotic Systems
% Final Project 

% Simulates quadcopter with inverted pendulum
% TO RUN:
%   1. Edit create_trajectoy to desired path
%   2. Set desired simulation time
%   3. Run script

% Note: Some functions have been commented out for ease of use (such as
% animation saving, etc.)

clc; clear; 
close all;
addpath('common_functions')

% Define Variables of Quadrotor and Pendulum
g = 9.8;
M_r = 1.5;
m_p = 0.5;
L = 0.6;
I_x = 5.0e3;
I_y = 5.0e3;
I_z = 5.0e3;

% Simulation Parameters
tf = 10;
t = 0:0.05:tf;

% Create Trajectory
[traj, vel, accel] = create_trajectory('fig8',1.5,length(t),0, tf);
start = traj(:,1);
goal = traj(:,end);

% Create Open Loop Model
[A, B, C, D , x0] = olSys(g, M_r, m_p, L, I_x, I_y, I_z, start, goal);
ol_Sys = ss(A, B, C, D);

% Create Control Law
G = optimalLQR(A, B, C, D);

% Create Closed Loop System
Ac = A-B*G;
cl_Sys = ss(Ac, B, C, D);

% Construct Input Vector
u = [zeros(4, length(t)); traj(1,:); traj(2,:); traj(3,:); vel(1,:); vel(2,:); vel(3,:); accel(1,:); accel(2,:); accel(3,:)];

% Simulate Closed Loop System
[y, t, x] = lsim(cl_Sys, u, t, x0); % Closed Loop System Clearly Stable
inpt = G*x';

% Plot Simulation Results
figure(1);
subplot(3, 1, 1);
plot(t, y(:, 1), 'r', t, y(:, 2), 'g', t, y(:, 3), 'b'); %, t, inpt, '--');
title('Quadrotor Position');
legend('X', 'Y', 'Z');
ylabel('Displacement (Meters)');
xlabel('Time (seconds)');
subplot(3, 1, 2);
plot(t, y(:, 6)*180/pi, 'r', t, y(:, 7)*180/pi, 'g', t, y(:, 8)*180/pi, 'b');
title('Quadrotor Orientation');
legend('Roll', 'Pitch', 'Yaw');
ylabel('Angle (deg)');
xlabel('Time (sec)');
subplot(3, 1, 3);
plot(t, y(:, 4), 'r', t, y(:, 5)*180/pi, 'g');
title('Pendulum Angle and Position');
legend('Position', 'Angle');
ylabel('Magnitude (deg/meters)');
xlabel('Time (sec)');
axis tight;

% Plot Trajectory Tracking Error
figure(2)
subplot(3, 1, 1);
plot(t, y(:, 1)-traj(1,:)');
title('Trajectory Tracking Error')
xlabel('Time (sec)');
ylabel('x error')
subplot(3, 1, 2);
plot(t, y(:, 2)-traj(2,:)');
xlabel('Time (sec)');
ylabel('y error')
subplot(3, 1, 3);
plot(t, y(:, 3)-traj(3,:)');
xlabel('Time (sec)');
ylabel('z error')

% Create Animation
filename = 'E:\MAE6245-final-project\media\1dofpend_helix.gif';

h = figure(3);
% set(gcf, 'Position', [200, 200, 1000, 700])
x_g = [y(1, 1), y(1, 2), y(1, 3)];
eul = [y(1, 6), y(1, 7), y(1, 8)];
draw_quadrotor(x_g, eul)
f = getframe(gcf);
[im,map] = rgb2ind(f.cdata,256,'nodither');
im(1,1,1,length(t)) = 0;

for i = 1:length(t)
    x_g = [y(i, 1), y(i, 2), y(i, 3)];
    eul = [y(i, 6), y(i, 7), y(i, 8)];
    
    % Draw Quadrotor, Pendulum, and Trajectory
    draw_quadrotor(x_g, eul)
    th = pi/2 - y(i,5);
    p_pend = x_g + [L*cos(th), 0, L*sin(th)]; % relative to quad COM
    draw_vector(x_g, p_pend,'r')
    plot3(traj(1,:),traj(2,:),traj(3,:),'r.')
    axis([-2.5 2.5 -2 2 -1 1])
%     view(-0, 0)
    drawnow
    
    % Save animation data
    f = getframe(gcf);
    im(:,:,1,i) = rgb2ind(f.cdata,map,'nodither');
    
    % Display Controller Input
    disp('Controller Input:');
    disp(-G*x(i, :)');
end

% Write Animation Data to File
imwrite(im,map,filename,'DelayTime',0,'LoopCount',inf)
