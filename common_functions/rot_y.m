% MAE 3184 - Spring 2017
% Prelab #1 written by Chris Poole

function R_y = rot_y(theta)
%
% function R_y = rot_y(theta)
%
% Takes an input angle theta (in radians) and produces a
% rotation matrix around the y-axis

R_y = [cos(theta), 0, sin(theta); 0, 1, 0; -sin(theta), 0, cos(theta)];

end