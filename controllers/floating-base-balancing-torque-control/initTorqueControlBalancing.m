%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% /**
%  * Copyright (C) 2016 CoDyCo
%  * @author: Daniele Pucci, Gabriele Nava
%  * Permission is granted to copy, distribute, and/or modify this program
%  * under the terms of the GNU General Public License, version 2 or any
%  * later version published by the Free Software Foundation.
%  *
%  * A copy of the license can be found at
%  * http://www.robotcub.org/icub/license/gpl.txt
%  *
%  * This program is distributed in the hope that it will be useful, but
%  * WITHOUT ANY WARRANTY; without even the implied warranty of
%  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
%  * Public License for more details
%  */
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% NOTE: THIS SCRIPT IS RUN AUTOMATICALLY WHEN THE USER STARTS THE ASSOCIATED
% SIMULINK MODEL. NO NEED TO RUN THIS SCRIPT EVERY TIME.
clearvars -except sl_synch_handles simulinkStaticGUI
clc

% Add path to local source code
addpath('./src/')

%% GENERAL SIMULATION INFO
%
% If you are simulating the robot with Gazebo, remember that it is required
% to launch Gazebo as follows:
% 
%     gazebo -slibgazebo_yarp_clock.so
% 
% and properly set the environmental variable YARP_ROBOT_NAME in the .bashrc.

% Simulation time in seconds. For long simulations, put an high number 
% (not inf) for allowing automatic code generation
Config.SIMULATION_TIME = 600000;

% Controller period [s]
Config.tStep           = 0.01;

%% PRELIMINARY CONFIGURATION
%
% DEMO_TYPE: defines the kind of demo that will be performed.
% 
DEMO_TYPE                     = 'COORDINATOR';
ROBOT_1_MODEL                 = 'iCubGazeboV2_5';
ROBOT_1_NAME                  = 'iCub1';
ROBOT_2_MODEL                 = 'iCubGazeboV2_5';
ROBOT_2_NAME                  = 'iCub2';

% Config.SCOPES: debugging scopes activation
Config.SCOPES_GAIN_SCHE_INFO  = false;
Config.SCOPES_MAIN            = true;
Config.SCOPES_QP              = true;

% DATA PROCESSING
% Save the Matlab workspace after stopping the simulation
Config.SAVE_WORKSPACE         = false;

% Verify that the integration time has been respected during the simulation
Config.CHECK_INTEGRATION_TIME = true;
Config.PLOT_INTEGRATION_TIME  = false;

% Run robot-specific configuration parameters
run(strcat('app/robots/',ROBOT_1_MODEL,'/configRobot.m')); 
run(strcat('app/robots/',ROBOT_1_MODEL,'/configStateMachine.m')); 
run(strcat('app/robots/',ROBOT_1_MODEL,'/gainsAndReferences.m')); 
WBTConfigRobot_1 = WBTConfigRobot.copy;
WBTConfigRobot_1.RobotName = ROBOT_1_NAME;

if (not(strcmp(ROBOT_1_MODEL, ROBOT_2_MODEL)))
    run(strcat('app/robots/',ROBOT_2_MODEL,'/configRobot.m')); 
    run(strcat('app/robots/',ROBOT_2_MODEL,'/configStateMachine.m')); 
    run(strcat('app/robots/',ROBOT_2_MODEL,'/gainsAndReferences.m')); 
end
WBTConfigRobot_2 = WBTConfigRobot.copy;
WBTConfigRobot_2.RobotName = ROBOT_2_NAME;

% Run board configuration parameters
run('app/configBoard.m'); 