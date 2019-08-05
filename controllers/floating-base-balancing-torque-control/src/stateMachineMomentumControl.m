function  [w_H_b, pos_CoM_des, feetContactStatus, KP_CoM_diag, KD_CoM_diag, state, smoothingTimeCoM, w_H_l_hand_des, w_H_r_hand_des, K_task_space] = ...
              stateMachineMomentumControl(pos_CoM_0, w_H_l_hand_0, w_H_r_hand_0, ...
                                          time, l_sole_H_b, StateMachine, Gain, Config)

    % STATEMACHINEMOMENTUMCONTROL generates the references for performing
    %                             two demos:
    %
    %                             YOGA: (highly dynamic movements) on one
    %                             foot and two feet;
    %
    %                             COORDINATOR: balancing on two feet while
    %                             performing left-right oscillations.
    
    %% --- Initialization ---

    persistent currentState;
    persistent t_switch;
    persistent w_H_fixedLink;
    persistent yogaMovesetCounter;
    
    if isempty(currentState) 
        
        currentState = StateMachine.initialState;        
    end
    if isempty(t_switch)
    
        t_switch = 0;
    end
    if  isempty(w_H_fixedLink)
        
        w_H_fixedLink = eye(4);
    end
    if isempty(yogaMovesetCounter)
        
        yogaMovesetCounter = 1;
    end
    
    % zero value
    pos_l_hand_0 = w_H_l_hand_0(1:3,4);
    pos_r_hand_0 = w_H_r_hand_0(1:3,4);
    
    % initialize outputs
    pos_CoM_des       = pos_CoM_0;
    feetContactStatus = [1; 1];
    w_H_b             = eye(4);
    w_H_l_hand_des    = w_H_l_hand_0;
    w_H_r_hand_des    = w_H_r_hand_0;

    %% STATE 1: TWO FEET BALANCING
    if currentState == 1 
        
        w_H_b = w_H_fixedLink * l_sole_H_b;

        if time > Config.noOscillationTime
            Amplitude = Config.amplitudeOfOscillation;
        else
            Amplitude = 0;
        end

        frequency   = Config.frequencyOfOscillation;
        pos_CoM_des = pos_CoM_0 - abs(Amplitude*sin(2*pi*frequency*time)*Config.directionOfOscillation);
        pos_l_hand_des = pos_l_hand_0 - abs(Amplitude*sin(2*pi*frequency*time)*Config.directionOfOscillation);
        pos_r_hand_des = pos_r_hand_0 - abs(Amplitude*sin(2*pi*frequency*time)*Config.directionOfOscillation);
        
        w_H_l_hand_des(1:3,4) = pos_l_hand_des;
        w_H_r_hand_des(1:3,4) = pos_r_hand_des;
    end
    
    smoothingTimeCoM = StateMachine.CoMSmoothingTime(currentState);
    
    % update gain matrices
    KP_CoM_diag      = Gain.KP_CoM(currentState,:);   
    KD_CoM_diag      = Gain.KD_CoM(currentState,:); 
    K_task_space     = Gain.K_task_space(currentState,:);
    
    % update current state
    state            = currentState;  
end