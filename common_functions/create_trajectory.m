function [traj, vel, accel] = create_trajectory(shape, size, num_points,z, tf)
    if(strcmp(shape,'circle'))
        th = linspace(0, 2*pi, num_points);
        x = size*cos(th);
        y = size*sin(th);
    end
    if(strcmp(shape,'fig8'))
        th = linspace(0, 2*pi, num_points);
        x = size*cos(th);
        y = size*sin(2*th);
    end
    z = z*ones(1,length(th));
    traj = [x;y;z];
    vel = zeros(3, 1);
    dt = tf/num_points;
    for i = 1:length(z)-1
        vel_ = (traj(:, i+1)-traj(:, i))/dt;
        vel = [vel, vel_];
    end
    accel = zeros(3, 1);
    for i = 1:length(vel)-1
        accel_ = (vel(:, i+1)-vel(:, i))/dt;
        accel = [accel, accel_];
    end
end