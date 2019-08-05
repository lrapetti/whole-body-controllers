% GAINSANDREFERENCES compute gains matrices, references and other control
%                    related quantities for each state of the state machine.

%% --- Initialization ---
  
% CoM gains
Gain.KP_CoM = [50  50   20];   % state ==  1  TWO FEET BALANCING

Gain.KD_CoM = 2*sqrt(Gain.KP_CoM)/20;

% Angular momentum gains
Gain.KI_AngularMomentum = 0.25 ;
Gain.KP_AngularMomentum = 2*sqrt(Gain.KI_AngularMomentum);

% Task space gains
%                    % LIN %%  ANG %%
Gain.K_task_space = [     100,     50];

%% Smoothing times

% Smoothing time gain scheduling
Config.SmoothingTimeGainScheduling = 2;

% Smoothing time CoM references
StateMachine.CoMSmoothingTime      = 1;

% Smoothing time for joints references 
StateMachine.scaleFactorSmoothingTime = 0.9;
                                
%% CoM delta

% To be summed to the reference CoM position
StateMachine.CoM_delta  = [0.0,  0.00,  0.0 ];   %% NOT USED

%% References for CoM and hands trajectory
Config.noOscillationTime       = 0;   
Config.directionOfOscillation  = [0;0;1];
Config.amplitudeOfOscillation  = 0.15; % [m]  
Config.frequencyOfOscillation  = 0.1;  % [Hz]