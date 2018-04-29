% Scott Barnes & Chris Poole
% MAE 6245: Robotic Systems
% Final Project 

% Draws quadrotor in global frame using the translation of the center of
% mass and rotations about the principle axes using Euler angles

% Inputs:
% x_g = global position [x,y,z]
% eangles = roll (gamma), pitch (beta), yaw (alpha) in radians

function draw_quadrotor(x_g, eangles)

v = -eangles(1);
u = -eangles(2);
w = eangles(3);

% Calculate Drawing Parameters
R_quad = rot_z(w)*rot_y(v)*rot_x(u);
R_e_q = [0 1  0; 1 0 0; 0 0 -1];

L = 0.25; % arm length in meters
arm_vects = [L 0 0; 0 L 0; -L 0 0; 0 -L 0];
prop_points = linspace(0, 2*pi, 100);
prop_rad = 0.1;
for i = 1:length(prop_points)
    prop_vects(i,:) = R_e_q*R_quad*[prop_rad*cos(prop_points(i)), prop_rad*sin(prop_points(i)), 0]';
end

% Execute Drawing
clf
plot3(x_g(1), x_g(2), x_g(3), 'r.', 'MarkerSize',15)
hold on
grid on
for i = 1:length(arm_vects)
    arm_vects_rot(:,i) = R_e_q*R_quad*arm_vects(i,:)';
    x_prop = x_g + arm_vects_rot(:,i)';
    draw_vector(x_g, x_prop, 'b')
    plot3(x_prop(1) + prop_vects(:,1), x_prop(2) + prop_vects(:,2), x_prop(3) + prop_vects(:,3), 'LineWidth',2)
end
axis equal
xlabel('x')
ylabel('y')
zlabel('z')

end

