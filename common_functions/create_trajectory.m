function [traj, vel, accel] = create_trajectory(shape, size, num_points,z, tf)
    if(strcmp(shape,'origin'))
        x = zeros(1,num_points);
        y = zeros(1,num_points);
        z = zeros(1,num_points);
    end
    if(strcmp(shape,'circle'))
        th = linspace(0, 2*pi, num_points);
        x = size*cos(th);
        y = size*sin(th);
        z = z*ones(1,length(th));
    end
    if(strcmp(shape,'fig8'))
        th = linspace(0, 2*pi, num_points);
        x = size*cos(th);
        y = 0.5*size*sin(2*th);
        z = z*ones(1,length(th));
    end
    if(strcmp(shape,'helix'))
        th = linspace(0, 2*pi, num_points);
        x = 0.8*size*cos(2*th);
        y = 0.8*size*sin(2*th);
        z = 0.08*th+z;
    end
    traj = [x;y;z];
    vel = [];
    dt = tf/num_points;
    for i = 1:length(z)-1
        vel_ = (traj(:, i+1)-traj(:, i))/dt;
        vel = [vel, vel_];
    end
    vel = [vel, vel_];
    accel = [];
    for i = 1:length(vel)-1
        accel_ = (vel(:, i+1)-vel(:, i))/dt;
        accel = [accel, accel_];
    end
    accel = [accel, zeros(3, 1)];
end