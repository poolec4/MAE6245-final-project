% Scott Barnes & Chris Poole
% MAE 6245: Robotic Systems
% Final Project 

% Takes an input angle theta (in radians) and produces a
% rotation matrix around the z-axis

function R_z = rot_z(theta)

R_z = [cos(theta), -sin(theta), 0; sin(theta), cos(theta), 0; 0, 0, 1];

end