% Scott Barnes & Chris Poole
% MAE 6245: Robotic Systems
% Final Project 

% Takes an input angle theta (in radians) and produces a
% rotation matrix around the x-axis

function R_x = rot_x(theta)

R_x = [1, 0, 0; 0, cos(theta), -sin(theta); 0, sin(theta), cos(theta)];

end