clear;
close all;

load('./old_mat_files/human_joint_ref_exp.mat');
% load('dummy.mat');

time      = human_jointData.time;
torso     = human_jointData.signals(1).values; % 3 DoF
left_arm  = human_jointData.signals(2).values; % 4 DoF
right_arm = human_jointData.signals(3).values; % 4 DoF
left_leg  = human_jointData.signals(4).values; % 6 DoF
right_leg = human_jointData.signals(5).values; % 6 DoF

ref = [torso left_arm right_arm left_leg right_leg];
ref_hands = [torso left_arm right_arm];

joint_ref = timeseries(ref,time);
joint_ref_hands = timeseries(ref_hands,time);

%%Saving the extracted joint positions to the workspace
save joint_ref -v7.3 joint_ref
save joint_ref_hands -v7.3 joint_ref_hands


