
function [A, B, C, D, x0] = olSys(g, M_q, m_p, L, I_x, I_y, I_z)
    % Create A Matrix
    A = zeros(16, 16);
    A(1, 4) = 1;
    A(2, 5) = 1;
    A(3, 6) = 1;
    A(4, 8) = -g;
    A(5, 7) = g;
    A(7, 10) = 1;
    A(8, 11) = 1;
    A(9, 12) = 1;
    A(13, 15) = 1;
    A(14, 16) = 1;
    A(15, 8) = -g;
    A(15, 14) = -m_p*g/M_q;
    A(16, 8) = g/L;
    A(16, 14) = (m_p+M_q)*g/(M_q*L);
    % B Matrix
    B = zeros(16, 4);
    B(6, 1) = 1/M_q;
    B(10, 2) = 1/I_x;
    B(11, 3) = 1/I_y;
    B(12, 4) = 1/I_z;
    % C Matrix
    C = zeros(5, 16);
    C(1, 1) = 1;
    C(2, 2) = 1;
    C(3, 3) = 1;
    C(4, 13) = 1;
    C(5, 14) = 1;
    C(6, 7) = 1;
    C(7, 8) = 1;
    C(8, 9) = 1;
    % D Matrix
    D = 0;
    % Initial State
    x0 = zeros(1, 16);
    x0(14) = pi/8; % Offset Starting Angle
end