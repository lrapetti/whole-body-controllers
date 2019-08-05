function jointVel_des  = solveInverseDifferentialKinematics(lin_vel_l_hand, ang_vel_l_hand, lin_vel_r_hand, ang_vel_r_hand, J)

% vel_hands = [ lin_vel_l_hand; lin_vel_r_hand ; zeros(6,1); zeros(6,1)];
% J_hands = [ J_l_hand(1:3, 1:end); J_r_hand(1:3, 1:end); J_l_sole(:,1:end); J_r_sole(:,1:end)];

vel_hands = [ zeros(6,1); zeros(6,1); lin_vel_l_hand; ang_vel_l_hand; lin_vel_r_hand; ang_vel_r_hand];

stateVel_des = pinv(J, 1.0000e-03) * vel_hands;

jointVel_des = stateVel_des(7:end);

end