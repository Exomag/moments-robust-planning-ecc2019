function params = loadParameters()
    % Problem
    params.HORIZON_TIME = 4;
    params.SAMPLING_TIME = 0.4;
    params.HORIZON = params.HORIZON_TIME / params.SAMPLING_TIME;

    % Road dimensions
    params.LANE_WIDTH = 3.5;

    % Ego vehicle
    params.CAR_LENGTH = 4.5;
    params.CAR_WIDTH = 2.5;
    params.EGO_X0 = -5;
    params.EGO_Y0 = -1.75;
    params.EGO_XDOT0 = 50 / 3.6;
    params.EGO_YDOT0 = 0;

    % Adversary vehicle
    params.ADV_LENGTH = 4.5;
    params.ADV_WIDTH = 2.5;
    params.ADV_X0 = 44;
    params.ADV_Y0 = params.LANE_WIDTH / 2;
    params.ADV_VEL0 = 22 / 3.6;
    params.ADV_THETA0 = 0;

    % Position constraints
    params.EGO_XMIN = -10;
    params.EGO_XMAX = 100 + 4 * params.CAR_LENGTH;
    params.EGO_YMIN = -params.LANE_WIDTH;
    params.EGO_YMAX = 0;

    % Velocity/acceleration box constraints
    params.MIN_XSPEED = 0 / 3.6;
    params.MAX_XSPEED = 80 / 3.6;
    params.MIN_YSPEED = -20 / 3.6;
    params.MAX_YSPEED = 20 / 3.6;
    params.MIN_XACCEL = -10;
    params.MAX_XACCEL = 3;
    params.MIN_YACCEL = -5;
    params.MAX_YACCEL = 5;

    % Velocity/acceleration coupled constraints
    params.A_VELOC_CONSTR = inv(diag([40 / 3.6, 20 / 3.6]));
    params.B_VELOC_CONSTR = [40 / 3.6; 0];
    params.A_ACCEL_CONSTR = zeros(2, 2);
    params.B_ACCEL_CONSTR = [6.5; 0];

    % Dynamics
    params.A_c = [0, 1, 0, 0; 0, 0, 0, 0; 0, 0, 0, 1; 0, 0, 0, 0];
    params.B_c = [0, 0; 1, 0; 0, 0; 0, 1];
    params.A = eye(4) + params.SAMPLING_TIME * params.A_c;
    params.B = params.SAMPLING_TIME * params.B_c;
    params.sys = c2d(ss(params.A_c, params.B_c, zeros(2, 4), zeros(2)), params.SAMPLING_TIME);
    params.A = params.sys.A;
    params.B = params.sys.B;

    % Objective parameters
    params.Q = 1;

    % Chance constraint parameters
    params.beta = 1e-3;
    params.epsilon = 0.05;
    params.N_s = 1000;

    % Other parameters
    params.M = 2000;
end
