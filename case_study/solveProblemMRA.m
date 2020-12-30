function [solData, solYALMIP, objective] = solveProblemMRA(params, paramOpt)
    % Problem parameters
    SAMPLING_TIME = params.SAMPLING_TIME;
    HORIZON = params.HORIZON;
    CAR_LENGTH = params.CAR_LENGTH;
    CAR_WIDTH = params.CAR_WIDTH;
    EGO_XMIN = params.EGO_XMIN;
    EGO_XMAX = params.EGO_XMAX;
    EGO_YMIN = params.EGO_YMIN;
    EGO_YMAX = params.EGO_YMAX;
    MIN_XSPEED = params.MIN_XSPEED;
    MAX_XSPEED = params.MAX_XSPEED;
    MIN_YSPEED = params.MIN_YSPEED;
    MAX_YSPEED = params.MAX_YSPEED;
    MIN_XACCEL = params.MIN_XACCEL;
    MAX_XACCEL = params.MAX_XACCEL;
    MIN_YACCEL = params.MIN_YACCEL;
    MAX_YACCEL = params.MAX_YACCEL;    
    A_VELOC_CONSTR = params.A_VELOC_CONSTR;
    B_VELOC_CONSTR = params.B_VELOC_CONSTR;
    A_ACCEL_CONSTR = params.A_ACCEL_CONSTR;
    B_ACCEL_CONSTR = params.B_ACCEL_CONSTR;
    A = params.A;
    B = params.B;
    Q = params.Q;
    M = params.M;
    
    % Optimization parameters
    mean_ineq_coeff = paramOpt.mean_ineq_coeff;
    var_ineq_coeff = paramOpt.var_ineq_coeff;
    r1 = paramOpt.r1;
    r2 = paramOpt.r2;
    eps_single = paramOpt.eps_single;
    x_0 = paramOpt.x_0;
    
    % Variables
    x_var = sdpvar(4,HORIZON+1);
    u_var = sdpvar(2,HORIZON);
    t_adv = binvar(4,HORIZON,5);

    % Objective
    objective = - Q * x_var(1,end);

    % Equality constraints
    constraints = (x_var(:,1) == x_0);
    for i = 1 : HORIZON
        constraints = [constraints, x_var(:,i+1) == A*x_var(:,i) + B*u_var(:,i)];
    end

    % Inequality constraints
    for i = 1 : HORIZON
        % Position
        constraints = [constraints, EGO_XMIN + CAR_LENGTH/2 <= x_var(1,i+1) <= EGO_XMAX - CAR_LENGTH/2];
        constraints = [constraints, EGO_YMIN + CAR_WIDTH/2 <= x_var(3,i+1) <= EGO_YMAX - CAR_WIDTH/2];

        % Velocity
        constraints = [constraints, MIN_XSPEED <= x_var(2,i+1) <= MAX_XSPEED];
        constraints = [constraints, MIN_YSPEED <= x_var(4,i+1) <= MAX_YSPEED];
        constraints = [constraints, norm(A_VELOC_CONSTR * (x_var([2,4],i+1) - B_VELOC_CONSTR), 1) <= 1];

        % Acceleration
        constraints = [constraints, MIN_XACCEL <= u_var(1,i) <= MAX_XACCEL];
        constraints = [constraints, MIN_YACCEL <= u_var(2,i) <= MAX_YACCEL];
        constraints = [constraints, norm(A_ACCEL_CONSTR * (u_var(:,i) + B_ACCEL_CONSTR), 1) <= 1];

        % Truck
        x_tilde = [x_var(1,i+1) ; x_var(3,i+1) ; 1];
        for j = 1 : 4
            for k = 1 : 5
                if k == 1 
                    temp_x = x_tilde + 1/2*[CAR_LENGTH ; CAR_WIDTH; 0];
                elseif k == 2
                    temp_x = x_tilde + 1/2*[CAR_LENGTH ; -CAR_WIDTH ; 0];
                elseif k == 3
                    temp_x = x_tilde + 1/2*[-CAR_LENGTH ; CAR_WIDTH ; 0];
                elseif k == 4
                    temp_x = x_tilde + 1/2*[-CAR_LENGTH ; -CAR_WIDTH ; 0];
                elseif k == 5
                    temp_x = x_tilde;
                end
                constraints = [constraints, norminv(1-eps_single*2) * sqrt(temp_x' * (var_ineq_coeff{i+1,j}+r2(i,j)*eye(3)) * temp_x) + r1(i,j) * sqrt(temp_x' * temp_x) <= mean_ineq_coeff{i+1,j} * temp_x + M * t_adv(j,i,k)];
                constraints = [constraints, sum(t_adv(:,i,k)) <= 3];
            end
        end
    end

    % Solve
    options = sdpsettings('verbose', 0);
    solYALMIP = optimize(constraints, objective, options);

    % Solution data
    solData.t_axis = SAMPLING_TIME * (0 : HORIZON);
    solData.x_var = zeros(4,HORIZON+1);
    solData.u_var = zeros(2,HORIZON);
    solData.t_adv = zeros(4,HORIZON,5,'logical');
    if solYALMIP.problem == 0
        solData.x_val = value(x_var);
        solData.u_val = value(u_var);
        solData.t_adv_val = value(t_adv);
    else
        disp('Hmm, something went wrong!');
        yalmiperror(solYALMIP.problem)
        return
    end
end
