% Scott Barnes & Chris Poole
% MAE 6245: Robotic Systems
% Final Project 

%   Determines if a linear system is controllable. 
% Inputs:
%   A: A Matrix of Linear System
%   B: B Matrix of Linear System
% Output:
%   cntrble: Boolean value, identifies controllability

function cntrble = is_controllable(A, B)
    % Cr = [B A*B A^2*B A^3*B];
    Cr = [];
    for i = 1:size(A, 1)
        Cr = [Cr, A^(i-1)*B];
    end
    disp('Rank of CR: ');
    disp(rank(Cr));
    c = rank(Cr);
    if c == size(A, 1)
        cntrble = 'true';
        disp('System is controllable');
    else
        cntrble = 'false';
        disp('System is not controllable');
    end
end