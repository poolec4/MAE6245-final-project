% MAE 3184 - Spring 2017
% Prelab #1 written by Chris Poole

function R_x = rot_x(theta)
%
% function R_x = rot_x(theta)
%
% Takes an input angle theta (in radians) and produces a
% rotation matrix around the x-axis

R_x = [1, 0, 0; 0, cos(theta), -sin(theta); 0, sin(theta), cos(theta)];

end