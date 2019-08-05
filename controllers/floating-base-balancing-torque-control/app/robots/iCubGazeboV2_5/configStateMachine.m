% CONFIGSTATEMACHINE configures the state machine (type of demo, velocity
%                    of the demo, repeat movements, and so on).

%% --- Initialization ---
% If equal to one, the desired values of the center of mass are smoothed internally 
Config.SMOOTH_COM_DES       = true;   

% If equal to one, the desired values of the postural tasks are smoothed internally 
Config.SMOOTH_JOINT_DES     = true;   

% max unsigned difference between two consecutive (measured) joint positions for emergency stop, 
% i.e. delta_qj = abs(qj(k) - qj(k-1))
Sat.maxJointsPositionDelta  = 15*pi/180; % [rad] 

%% Regularization parameters
Reg.pinvDamp_baseVel        = 1e-7;
Reg.pinvDamp                = 1; 
Reg.pinvTol                 = 1e-5;
Reg.KP_postural             = 0.1;
Reg.KD_postural             = 0;
Reg.HessianQP               = 1e-7;    

%% State Machine configuration
% initial state for state machine
StateMachine.initialState                 = 1;

%% Constraints for QP for balancing

% The friction cone is approximated by using linear interpolation of the circle. 
% So, numberOfPoints defines the number of points used to interpolate the circle 
% in each cicle's quadrant
numberOfPoints               = 4;  
forceFrictionCoefficient     = 1/3;    
torsionalFrictionCoefficient = 1/75;
fZmin                        = 10;

% physical size of the foot                             
feet_size                    = [-0.07  0.12 ;    % xMin, xMax
                                -0.045 0.05 ];   % yMin, yMax  
                                                    
% Compute contact constraints (friction cone, unilateral constraints)
[ConstraintsMatrix, bVectorConstraints] = wbc.computeRigidContactConstraints ...
    (forceFrictionCoefficient, numberOfPoints, torsionalFrictionCoefficient, feet_size, fZmin);