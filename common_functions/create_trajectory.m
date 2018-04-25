function traj = create_trajectory(shape, size, num_points,z)
    if(strcmp(shape,'circle'))
        t = linspace(0, 2*pi, num_points);
        x = size*cos(t);
        y = size*sin(t);
    end
    if(strcmp(shape,'fig8'))
        t = linspace(0, 2*pi, num_points);
        x = size*cos(t);
        y = size*sin(2*t);
    end
    z = z*ones(1,length(t));
    traj = [x;y;z];
end