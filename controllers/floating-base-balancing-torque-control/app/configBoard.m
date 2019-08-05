% Inertia parameters (from board.sdf)
% the board frame is assumed to be z-up and x-y along the axis of the board
I = [0.0416 0.0    0.0;
     0.0    0.0416 0.0;
     0.0    0.0    0.0416];
m = 2;
g = 9.81;

ConfigBoard.MassMatrix = [   m  0.0 0.0    0.0   m*g   0.0;
                           0.0    m 0.0   -m*g   0.0   0.0;
                           0.0  0.0   m    0.0   0.0   0.0;
                           0.0 -m*g 0.0  I(1,1)  0.0   0.0;
                           m*g  0.0 0.0    0.0 I(2,2)  0.0;
                           0.0  0.0 0.0    0.0   0.0 I(3,3)];

% Dimension
ConfigBoard.size = [0.5 0.5 0.025];

