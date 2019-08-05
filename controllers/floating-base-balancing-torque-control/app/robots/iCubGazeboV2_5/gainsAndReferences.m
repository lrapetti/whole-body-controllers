% GAINSANDREFERENCES compute gains matrices, references and other control
%                    related quantities for each state of the state machine.

%% --- Initialization ---
  
% CoM gains
Gain.KP_CoM = [50  50   20];   % state ==  1  TWO FEET BALANCING

Gain.KD_CoM = 2*sqrt(Gain.KP_CoM)/20;

% Angular momentum gains
Gain.KI_AngularMomentum = 0.25 ;
Gain.KP_AngularMomentum = 2*sqrt(Gain.KI_AngularMomentum);

% Postural task gains
%                   %  TORSO  %%        LEFT ARM   %%       RIGHT ARM   %%         LEFT LEG           %%         RIGHT LEG           %% 
Gain.KP_postural = [10   30   20,  10   10    10    8,  10   10    10    8,  30   30   20   20   100 100,  30   50   30   60   100  100 ];

Gain.KD_postural = 2*sqrt(Gain.KP_postural(1,:))/20;

% Inverse kinematics gains
%           % LIN %%  ANG %%
Gain.K_IK = [     -40,     -40];

%% Smoothing times

% Smoothing time gain scheduling
Config.SmoothingTimeGainScheduling = 2;

% Smoothing time CoM references
StateMachine.CoMSmoothingTime      = 1;

% Smoothing time for joints references 
StateMachine.jointsSmoothingTime   = 1;
StateMachine.scaleFactorSmoothingTime = 0.9;
                                
%% CoM delta

% To be summed to the reference CoM position
StateMachine.CoM_delta  = [0.0,  0.00,  0.0 ];   %% NOT USED

%% Joint references
StateMachine.joints_references = [ zeros(1,ROBOT_DOF) ];  %% THIS REFERENCE IS IGNORED 

%% References for CoM and hands trajectory
Config.noOscillationTime       = 0;   
Config.directionOfOscillation  = [0;0;1];
Config.amplitudeOfOscillation  = 0.15; % [m]  
Config.frequencyOfOscillation  = 0.1;  % [Hz]