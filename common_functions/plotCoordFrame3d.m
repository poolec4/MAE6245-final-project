function plotCoordFrame3d(R_12, d_12, quiver_length)
    % Written by Chris Poole
    % plot_coord_frame3d(R_12(3x3), d_12(3x1), quiver_length)
    %
    %   Takes in a 3D rotation matrix and displacement (column) vector and plots the x, y, and z 
    %   unit vecors for the original and transformed frame.
    
    % Functions to draw the unit vectors for each axis
    drawUnitVectorX = @(u,O) quiver3(O(1),O(2),O(3),u(1,1),u(2,1),u(3,1),'.','color',[1 0 0],'AutoScaleFactor',quiver_length); 
    drawUnitVectorY = @(u,O) quiver3(O(1),O(2),O(3),u(1,2),u(2,2),u(3,2),'.','color',[0 1 0],'AutoScaleFactor',quiver_length);
    drawUnitVectorZ = @(u,O) quiver3(O(1),O(2),O(3),u(1,3),u(2,3),u(3,3),'.','color',[0 0 1],'AutoScaleFactor',quiver_length);
    
    % Defining frame 1 origin
    O_1 = zeros(3,1);
    u_1 = eye(3);
    
    % Perform transform from frame 1 to 2
    O_2 = d_12+O_1;
    u_2 = R_12*u_1;
    
    % Plot unit vectors for frame 1 and 2
%     drawUnitVectorX(u_1,O_1)
%     drawUnitVectorY(u_1,O_1)
%     drawUnitVectorZ(u_1,O_1)
    
    drawUnitVectorX(u_2,O_2)
    drawUnitVectorY(u_2,O_2)
    drawUnitVectorZ(u_2,O_2)
end