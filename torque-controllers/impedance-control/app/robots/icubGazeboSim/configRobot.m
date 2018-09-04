% Gains and parameters for impedance controller
Config.ON_GAZEBO = true;
ROBOT_DOF        = 1;

% Robot configuration for WBT3.0
WBTConfigRobot           = WBToolbox.Configuration;
WBTConfigRobot.RobotName = 'icubSim/jtcvc';
WBTConfigRobot.UrdfFile  = 'model.urdf';
WBTConfigRobot.LocalName = 'WBT';

%WBTConfigRobot.ControlBoardsNames = {'torso','left_arm','right_arm','left_leg','right_leg'};
% WBTConfigRobot.ControlledJoints   = {'torso_pitch','torso_roll','torso_yaw', ...
%                                      'l_shoulder_pitch','l_shoulder_roll','l_shoulder_yaw','l_elbow', ...
%                                      'r_shoulder_pitch','r_shoulder_roll','r_shoulder_yaw','r_elbow', ...
%                                      'l_hip_pitch','l_hip_roll','l_hip_yaw','l_knee','l_ankle_pitch','l_ankle_roll', ...
%                                      'r_hip_pitch','r_hip_roll','r_hip_yaw','r_knee','r_ankle_pitch','r_ankle_roll'};
WBTConfigRobot.ControlBoardsNames = {'left_leg'};
WBTConfigRobot.ControlledJoints   = {'l_ankle_roll'};

% Gains of the torque-to-current control
Kc = 0.0;
Kv = 0.0;
Kt = 1.0;

% References for the demo movements
if MOVING
    
    % Impedance gains
    Kp     = 10*diag(ones(1,ROBOT_DOF));
    Kd     = 2*sqrt(Kp)*0;
    
    AMPLS  = 5*ones(1,ROBOT_DOF);
    FREQS  = 0.25*ones(1,ROBOT_DOF);
   
else
    
    % Impedance gains
    Kp     = 0*diag(ones(1,ROBOT_DOF));
    Kd     = 0*diag(ones(1,ROBOT_DOF));
    
    AMPLS  = 0*ones(1,ROBOT_DOF);
    FREQS  = 0*ones(1,ROBOT_DOF);
end
    
if size(Kp,1) ~= ROBOT_DOF
    error('Dimension of Kp different from ROBOT_DOF')
end
