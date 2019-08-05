function [HessianMatrixTwoFeet, gradientTwoFeet, ConstraintsMatrixTwoFeet, bVectorConstraintsTwoFeet, ...
          tauModel, Sigma, Na, f_LDot] =  ...
              momentumBasedController(ConstraintsMatrix, bVectorConstraints, nu, M, h, L, w_H, J, JDot_nu, ...
                                         pos_CoM, J_CoM, KP_CoM, KD_CoM, K_task_space, desired_pos_vel_acc_CoM, w_H_l_hand_des, w_H_r_hand_des, Reg, Gain, Config )
    
    % MOMENTUMBASEDCONTROLLER implements a momentum-based whole body
    %                         balancing controller for humanoid robots.
    %
    % REFERENCES: G. Nava and F. Romano and F. Nori and D. Pucci, 
    %            "Stability Analysis and Design of Momentum Based Controllers for Humanoid Robots",
    %             Available at: https://ieeexplore.ieee.org/document/7759126/

    %% --- Initialization ---
    
    % Extract concatenated matrices
    w_H_l_sole = w_H(1:4,:);
    w_H_r_sole = w_H(5:8,:);
    w_H_l_hand = w_H(9:12,:);
    w_H_r_hand = w_H(13:16,:);
    J_l_sole = J(1:6,:);
    J_r_sole = J(7:12,:);
    % J_l_hand = J(13:18,:);
    % J_r_hand = J(19:24,:);
    JDot_l_sole_nu = JDot_nu(1:6);
    JDot_r_sole_nu = JDot_nu(7:12);
    % JDot_l_hand_nu = JDot_nu(13:18);
    % JDot_r_hand_nu = JDot_nu(19:24);


    % Compute the momentum rate of change. The momentum rate of change
    % equals the summation of the external forces and moments, i.e.:
    %
    %    LDot = A*f + f_grav (1)
    %
    % where A is the matrix mapping the forces and moments into the
    % momentum equations, f_grav is the gravity force, f is a vector stacking
    % all the external forces and moments acting on the robot as follows:
    %
    %    f = [f_left; f_right]
    %
    % where f_left are the forces and moments acting on the left foot and
    % f_right are the forces and moments acting on the right foot.
    
    % Compute the gravity force
    m             = M(1,1);
    gravAcc       = Config.GRAV_ACC;
    f_grav        = [zeros(2,1);
                    -m*gravAcc;
                     zeros(3,1)];
    
    % Compute matrix A in Eq. (1)
    pos_leftFoot  = w_H_l_sole(1:3,4);
    pos_rightFoot = w_H_r_sole(1:3,4);

    % Distance between the application points of the contact forces w.r.t. CoM
    r_left        = pos_leftFoot  - pos_CoM; 
    r_right       = pos_rightFoot - pos_CoM; 
    
    % Partition matrix A into the part that multiplies the left foot
    % wrenches and the right foot wrenches, i.e. A = [A_left, A_right]
    A_left        = [eye(3),           zeros(3);
                     wbc.skew(r_left), eye(3)];
    A_right       = [eye(3),            zeros(3);
                     wbc.skew(r_right), eye(3)];

    A             = [A_left, A_right]; 
    
    %% MOMENTUM CONTROL
    %
    % We would like to achieve a desired momentum's dynamics:
    %
    %    LDot_star = LDot_des - KP_momentum*(L-LDes) - KI_momentum*(intL-intLDes)
    % 
    % for the moment we are going not to consieder the integral error. 
    % Assume the contact forces and moments can be considered as control 
    % inputs of Eq. (1). Then, the problem is to find f such that:
    %
    %    LDot_star = A*f + f_grav (2)
    %
    % there is redundancy as there are more control inputs (12) than 
    % variables to control (6). Therefore one can write:
    %
    %    f = pinvA*(LDot_star - f_grav) + Na*f_0 (4)
    %
    % where pinvA is the pseudoinverse of matrix A and Na is its null space
    % projector. f_0 is a free variable that does not affect the momentum
    % dynamics Eq (1).
    
    % Gains mapping. 
    %
    %    KP_momentum = blkdiag(KD_CoM, KP_angMom)
    %    KD_momentum = blkdiag(KP_CoM, KI_angMom)
    %
    KP_angMom    = Gain.KP_AngularMomentum*eye(3);
    
    % Desired CoM dynamics (conseguently, linear momentum)
    vel_CoM      = J_CoM(1:3,:) * nu;
    acc_CoM_star = desired_pos_vel_acc_CoM(:,3) - KP_CoM*(pos_CoM - desired_pos_vel_acc_CoM(:,1)) - KD_CoM*(vel_CoM - desired_pos_vel_acc_CoM(:,2));

    % Desired momentum dynamics
    LDot_star    = [m * acc_CoM_star;
                   (-KP_angMom * L(4:end))];
    
             
    % we solve Eq (4) by means of the matrix pseudoinverse and. The QP is 
    % instead used to calculate the vector projected in the null space (f_0)
    % In particular, we choose f_0 in order to minimize the joint torques 
    % magnitude. To do so, it is necessary to write down the relationship 
    % between the joint torques and the contact forces starting from the 
    % dynamic equation.
    %
    %    tau = pinvLambda*(Jc*invM*(h - Jc^T*f) -JcDot_nu) + NLambda*tau_0 (6)
    %
    % where tau_0 is given by the following equation:
    %
    %    tau_0 = hs - Msb*invMb*hb + (Js^T - Msb*invMb*Jb^T)*f + u_0
    %
    % where we have:
    %
    %    M = [Mb, Mbs;    h = [hb;    Jc = [Jb, Js]
    %         Msb, Ms];        hs];
    %    
    % obtained by partitioning the dynamics in order to split the first
    % six rows and the remaining NDOF rows.
    %
    % u_0 instead are the feedback terms associated with the tasks space 
    % control objective, and therefore are given by the following 
    % expression:
    %
    %    u_0       = M*pinv(J)*(vDot_star - JDot_nu) + N_J*nuDot_0
    %    vDot_star = [  0_6
    %                   0_6
    %                  -K_linear_task_space*(pos_l_hand - pos_l_hand_des)
    %                  -K_rotational_task_space*skeevee(R_l_hand * (R_l_hand_des)^(-1))
    %                  -K_linear_task_space*(pos_r_hand - pos_r_hand_des)
    %                  -K_rotational_task_space*skeevee(R_r_hand * (R_r_hand_des)^(-1))
    %
    % assuming we want to minimize the joint velocities, we assume
    % nuDot_0=0.
    %
    % Now, let us rewrite Eq. (6) in order to isolate the terms which
    % depend on the contact forces:
    %
    %    tau = Sigma*f + tauModel  (7)   
    %
    % where Sigma    = -(pinvLambda*Jc*invM*Jc^T + NLambda*(Js^T - Msb*invMb*Jb^T))
    %
    %       tauModel = pinvLambda*(Jc*invM*h -JcDot_nu) + ...
    %                  NLambda*(hs - Msb*invMb*hb + u_0)
    %
    % Finally, we substitute Eq. (4) into Eq. (7) which gives:
    %
    %    tau = Sigma*pinvA*(LDot_star - f_grav) + Sigma*Na*f_0 + tauModel (8)
    %
    % minimizing the torques implies we would like to have tau = 0 in Eq.
    % (8) (note that it is not possible to achieve tau = 0 by choosing f_0)
    % 
    % It is possible to write down Eq. (8) as a QP problem, as we
    % did for Eq. (5):
    %
    %    f_0^T*Hessian*f_0 + f_0^T*gradient = 0 (9)
    % 
    % where Hessian  = transpose(Sigma*Na)*Sigma*Na 
    %       gradient = transpose(Sigma*Na)*(Sigma*pinvA*(LDot_star - f_grav) + tauModel)
    %
    % The associated QP formulation is now:
    %
    % f_0_star = argmin_f_0 |f_0^T*Hessian*f_0 + f_0^T*gradient|^2
    %
    %            s.t. C*f_0 < b
    %
    % Note that in this way we are assuming that the part of the contact
    % forces dedicated to stabilize the momentum dynamics, i.e. the term
    %
    %    f_LDot = pinvA*(LDot_star - f_grav)
    %
    % is does not violate the constraints.
    
    % Compute f_LDot   
    pinvA       = pinv(A, Reg.pinvTol); 
    f_LDot      = pinvA*(LDot_star - f_grav);
                
    % Null space of the matrix A            
    Na          = (eye(12,12) - pinvA*A);
    
    %% Compute Sigma and tauModel
    %
    
    % Feet jacobians
    NDOF        = size(J_l_sole(:,7:end),2);
    Jc          = [J_l_sole;      
                   J_r_sole];
                   
    % Jacobian derivative dot(Jc)*nu for the feet
    JcDot_nu    = [JDot_l_sole_nu;      
                   JDot_r_sole_nu];

    % Selector of actuated DoFs
    B           = [zeros(6,NDOF);
                   eye(NDOF,NDOF)];
                            
    % The mass matrix is partitioned as:
    %
    %   M = [ Mb,   Mbs
    %         Mbs', Ms ];  
    %
    % where: Mb  \in R^{6 x 6}
    %        Mbs \in R^{6 x 6+NDOF}
    %        Ms  \in R^{NDOF x NDOF}
    %
    Mb          = M(1:6,1:6);
    Mbs         = M(1:6,7:end);
    Ms          = M(7:end,7:end);
                 
    % Get matrix Sigma        
    Jc_invM     =  Jc/M;
    Lambda      =  Jc_invM*B;
    pinvLambda  =  wbc.pinvDamped(Lambda, Reg.pinvDamp); 
    NullLambda  =  eye(NDOF) - pinvLambda*Lambda;
    Sigma       = -(pinvLambda*Jc_invM*Jc' + NullLambda*(transpose(Jc(:,7:end)) -Mbs'/Mb*transpose(Jc(:,1:6))));
  
    % Get the vector tauModel
    % use desired frame pose for the hands, while for the feet we set
    % vDot_l_sole=vDot_r_sole=0 in order to use the lower joints
    vDot_l_hand     = [- K_task_space(1) * (w_H_l_hand(1:3,4) - w_H_l_hand_des(1:3,4) ); - K_task_space(2) * wbc.skewVee(w_H_l_hand(1:3,1:3)/w_H_l_hand_des(1:3,1:3))];
    vDot_r_hand     = [- K_task_space(1) * (w_H_r_hand(1:3,4) - w_H_r_hand_des(1:3,4) ); - K_task_space(2) * wbc.skewVee(w_H_r_hand(1:3,1:3)/w_H_r_hand_des(1:3,1:3))];
    vDot_star       = [zeros(12,1); vDot_l_hand; vDot_r_hand]; 
    
    u_0             =  Ms * wbc.pinvDamped(J(:,7:end), Reg.pinvDamp) * (vDot_star - JDot_nu); 
    tauModel        =  pinvLambda*(Jc_invM*h - JcDot_nu) + NullLambda*(h(7:end) - Mbs'/Mb*h(1:6) + u_0);
  
    %% QP parameters for two feet standing
    %
    % The control objective is the minimization of the joint torques 
    % through the redundancy of the contact forces. See Previous comments.

    % Get the inequality constraints matrices
    w_R_r_sole                = w_H_r_sole(1:3,1:3);
    w_R_l_sole                = w_H_l_sole(1:3,1:3);
    ConstraintMatrixLeftFoot  = ConstraintsMatrix * blkdiag(w_R_l_sole', w_R_l_sole');
    ConstraintMatrixRightFoot = ConstraintsMatrix * blkdiag(w_R_r_sole', w_R_r_sole');
    ConstraintsMatrixBothFeet  = blkdiag(ConstraintMatrixLeftFoot,ConstraintMatrixRightFoot);
    bVectorConstraintsBothFeet = [bVectorConstraints;bVectorConstraints];
    
    % The optimization problem Eq. (9) seeks for the redundancy of the external
    % wrench that minimize joint torques. Recall that the contact wrench can 
    % be written as:
    %
    %     f = f_LDot + Na*f_0 
    %
    % Then, the constraints on the contact wrench is of the form
    %
    %     ConstraintsMatrixBothFeet*f < bVectorConstraints
    %
    % which in terms of f0 is:
    %
    % ConstraintsMatrixBothFeet*Na*f0 < bVectorConstraints - ConstraintsMatrixBothFeet*f_LDot
    %
    ConstraintsMatrixTwoFeet  = ConstraintsMatrixBothFeet*Na;
    bVectorConstraintsTwoFeet = bVectorConstraintsBothFeet - ConstraintsMatrixBothFeet*f_LDot;
    
    % Evaluation of Hessian matrix and gradient vector for solving the
    % optimization problem Eq. (9)
    Sigma_Na                  = Sigma*Na;
    HessianMatrixTwoFeet      = Sigma_Na'*Sigma_Na + eye(size(Sigma_Na,2))*Reg.HessianQP;
    gradientTwoFeet           = Sigma_Na'*(tauModel + Sigma*f_LDot);
end