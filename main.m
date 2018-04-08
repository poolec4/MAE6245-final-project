% Scott Barnes & Chris Poole
% MAE 6245: Robotic Systems
% Final Project 
clc; clear; close all;
addpath('common_functions')

% Simulates quadcopter with inverted pendulum
% Define Variables
g = 9.8;
M_r = 100;
m_p = 1;
L = 1;
I_x = 10;
I_y = 10;
I_z = 10;
% Create Open Loop Model
[A, B, C, D , x0] = olSys(g, M_r, m_p, L, I_x, I_y, I_z);
ol_Sys = ss(A, B, C, D);
% Validate Controllability & Observability
% is_controllable(A, B);
% is_observable(A, C);
% Create Control Law
G = optimalLQR(A, B, C, D);
% Create Closed Loop System
Ac = A-B*G;
cl_Sys = ss(Ac, B, C, D);
% Simulate System
t = 0:0.1:10;
% u = [t; -t; ones(1, length(t)); -ones(1, length(t))];
% u = [ones(3, length(t)); zeros(1, length(t))];
u = zeros(4, length(t));
% Simulate Closed Loop System
[y, t, x] = lsim(cl_Sys, u, t, x0); % Closed Loop System Clearly Stable
inpt = G*x';
figure;
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
title('Pendulum Angle \& Position');
legend('Position', 'Angle');
ylabel('Magnitude (deg/meters)');
xlabel('Time (sec)');

filename = 'E:\MAE6245-final-project\media\1dofpend.gif';

h = figure;
x_g = [y(1, 1), y(1, 2), y(1, 3)];
eul = [y(1, 6), y(1, 7), y(1, 8)];
draw_quadrotor(x_g, eul)
axis([-5 10 -2 2 -2 2])
view(-20, 20)
f = getframe(gcf);
[im,map] = rgb2ind(f.cdata,256,'nodither');
im(1,1,1,length(t)) = 0;

for i = 1:length(t)
    x_g = [y(i, 1), y(i, 2), y(i, 3)];
    eul = [y(i, 6), y(i, 7), y(i, 8)];
    draw_quadrotor(x_g, eul)
    th = pi/2 - y(i,5);
    p_pend = x_g + [L*cos(th), 0, L*sin(th)]; % relative to quad COM
    draw_vector(x_g, p_pend,'r')
    axis([-5 10 -2 2 -2 2])
    view(-20, 20)
    
    drawnow
    f = getframe(gcf);
    im(:,:,1,i) = rgb2ind(f.cdata,map,'nodither');
end

imwrite(im,map,filename,'DelayTime',0,'LoopCount',inf)

% Plot Order: X_q, Y_q, Z_q, Y_p, Theta_p, Roll, Pitch, Yaw
% Simulate Open Loop System
% figure;
% lsim(ol_Sys, u, t) % Open Loop System Clearly Unstable