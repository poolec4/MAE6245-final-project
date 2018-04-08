% MAE 3184 - Spring 2017
% Prelab #1 written by Chris Poole

function R_z = rot_z(theta)
%
% function R_z = rot_z(theta)
%
% Takes an input angle theta (in radians) and produces a
% rotation matrix around the z-axis

R_z = [cos(theta), -sin(theta), 0; sin(theta), cos(theta), 0; 0, 0, 1];

end