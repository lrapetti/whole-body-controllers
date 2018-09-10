clear;
close all;

load('state4.mat');
% load('dummy.mat');

time      = human_jointData.time;
torso     = human_jointData.signals(1).values; % 3 DoF
left_arm  = human_jointData.signals(2).values; % 4 DoF
right_arm = human_jointData.signals(3).values; % 4 DoF
left_leg  = human_jointData.signals(4).values; % 6 DoF
right_leg = human_jointData.signals(5).values; % 6 DoF

ref = [torso left_arm right_arm left_leg right_leg];

joint_ref = timeseries(ref,time);

value = [torso(end,:) left_arm(end,:) right_arm(end,:) left_leg(end,:) right_leg(end,:)]

%%Saving the extracted joint positions to the workspace
% save joint_ref -v7.3 joint_ref


