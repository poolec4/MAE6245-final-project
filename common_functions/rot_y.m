% Scott Barnes & Chris Poole
% MAE 6245: Robotic Systems
% Final Project 

% Takes an input angle theta (in radians) and produces a
% rotation matrix around the y-axis

function R_y = rot_y(theta)

R_y = [cos(theta), 0, sin(theta); 0, 1, 0; -sin(theta), 0, cos(theta)];

end