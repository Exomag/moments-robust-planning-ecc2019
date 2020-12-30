function paramOpt = calcOptimizationParameters(params)
    % Parameters
    SAMPLING_TIME = params.SAMPLING_TIME;
    HORIZON = params.HORIZON;
    EGO_X0 = params.EGO_X0;
    EGO_Y0 = params.EGO_Y0;
    EGO_XDOT0 = params.EGO_XDOT0;
    EGO_YDOT0 = params.EGO_YDOT0;
    ADV_LENGTH = params.ADV_LENGTH;
    ADV_WIDTH = params.ADV_WIDTH;
    ADV_X0 = params.ADV_X0;
    ADV_Y0 = params.ADV_Y0;
    ADV_VEL0 = params.ADV_VEL0;
    ADV_THETA0 = params.ADV_THETA0;
    beta = params.beta;
    epsilon = params.epsilon;
    N_s = params.N_s;

    % Angular velocity realizations
    w = cell(1, N_s);
    for k = 1:N_s
        w{k}.theta = zeros(HORIZON+1, 1);
        for i = 1:HORIZON
            w{k}.theta(i) = pi / 2 / (HORIZON + 1) - 0.03 + min(0.06, pi/2-sum(w{k}.theta)-pi/2/(HORIZON + 1)+0.03) * rand;
        end
        w{k}.theta(HORIZON+1) = pi / 2 - sum(w{k}.theta);
        w{k}.v = -ADV_VEL0 * ones(HORIZON+1, 1);
    end
    paramOpt.w = w;

    % Truck trajectory realizations
    truck_realizations_y1 = zeros(HORIZON+1, N_s);
    truck_realizations_y2 = zeros(HORIZON+1, N_s);
    truck_realizations_theta = zeros(HORIZON+1, N_s);
    for k = 1:N_s
        pose_out = generateTruckTrajectory([ADV_X0; ADV_Y0; ADV_THETA0], SAMPLING_TIME, w{k});
        truck_realizations_y1(:, k) = pose_out(:, 1);
        truck_realizations_y2(:, k) = pose_out(:, 2);
        truck_realizations_theta(:, k) = -pose_out(:, 3);
    end
    paramOpt.truck_realizations_y1 = truck_realizations_y1;
    paramOpt.truck_realizations_y2 = truck_realizations_y2;
    paramOpt.truck_realizations_theta = truck_realizations_theta;

    % Truck state moments
    mean_truck_state = cell(HORIZON+1, 1);
    var_truck_state = cell(HORIZON+1, 1);
    for i = 1:HORIZON + 1
        d = [truck_realizations_y1(i, :)', truck_realizations_y2(i, :)', truck_realizations_theta(i, :)'];
        mean_truck_state{i} = 1 / N_s * sum(d);
        temp_sum = zeros(3, 3);
        for k = 1:N_s
            temp_sum = temp_sum + (d(k, :) - mean_truck_state{i})' * (d(k, :) - mean_truck_state{i});
        end
        var_truck_state{i} = 1 / N_s * temp_sum;
    end
    paramOpt.mean_truck_state = mean_truck_state;
    paramOpt.var_truck_state = var_truck_state;

    % Inequality coefficient moments
    coeff_realizations = cell(4, 3);
    for i = 1:size(coeff_realizations, 1)
        for j = 2:size(coeff_realizations, 2)
            coeff_realizations{i, j} = zeros(HORIZON+1, N_s);
        end
    end
    mean_ineq_coeff = cell(HORIZON+1, 4);
    var_ineq_coeff = cell(HORIZON+1, 4);
    for i = 1:HORIZON + 1
        for j = 1:4
            if j == 1
                coeff_realizations{j, 1}(i, :) = cos(truck_realizations_theta(i, :));
                coeff_realizations{j, 2}(i, :) = -sin(truck_realizations_theta(i, :));
                coeff_realizations{j, 3}(i, :) = -(cos(truck_realizations_theta(i, :)) .* truck_realizations_y1(i, :) - sin(truck_realizations_theta(i, :)) .* truck_realizations_y2(i, :) + 1 / 2 * ADV_LENGTH);
                d = [cos(truck_realizations_theta(i, :)'), -sin(truck_realizations_theta(i, :)'), -(cos(truck_realizations_theta(i, :)') .* truck_realizations_y1(i, :)' - sin(truck_realizations_theta(i, :)') .* truck_realizations_y2(i, :)' + 1 / 2 * ADV_LENGTH)];
            elseif j == 2
                coeff_realizations{j, 1}(i, :) = sin(truck_realizations_theta(i, :));
                coeff_realizations{j, 2}(i, :) = cos(truck_realizations_theta(i, :));
                coeff_realizations{j, 3}(i, :) = -(sin(truck_realizations_theta(i, :)) .* truck_realizations_y1(i, :) + cos(truck_realizations_theta(i, :)) .* truck_realizations_y2(i, :) + 1 / 2 * ADV_WIDTH);
                d = [sin(truck_realizations_theta(i, :)'), cos(truck_realizations_theta(i, :)'), -(sin(truck_realizations_theta(i, :)') .* truck_realizations_y1(i, :)' + cos(truck_realizations_theta(i, :)') .* truck_realizations_y2(i, :)' + 1 / 2 * ADV_WIDTH)];
            elseif j == 3
                coeff_realizations{j, 1}(i, :) = -cos(truck_realizations_theta(i, :));
                coeff_realizations{j, 2}(i, :) = sin(truck_realizations_theta(i, :));
                coeff_realizations{j, 3}(i, :) = -(-cos(truck_realizations_theta(i, :)) .* truck_realizations_y1(i, :) + sin(truck_realizations_theta(i, :)) .* truck_realizations_y2(i, :) + 1 / 2 * ADV_LENGTH);
                d = [-cos(truck_realizations_theta(i, :)'), +sin(truck_realizations_theta(i, :)'), -(-cos(truck_realizations_theta(i, :)') .* truck_realizations_y1(i, :)' + sin(truck_realizations_theta(i, :)') .* truck_realizations_y2(i, :)' + 1 / 2 * ADV_LENGTH)];
            elseif j == 4
                coeff_realizations{j, 1}(i, :) = -sin(truck_realizations_theta(i, :));
                coeff_realizations{j, 2}(i, :) = -cos(truck_realizations_theta(i, :));
                coeff_realizations{j, 3}(i, :) = -(-sin(truck_realizations_theta(i, :)) .* truck_realizations_y1(i, :) - cos(truck_realizations_theta(i, :)) .* truck_realizations_y2(i, :) + 1 / 2 * ADV_WIDTH);
                d = [-sin(truck_realizations_theta(i, :)'), -cos(truck_realizations_theta(i, :)'), -(-sin(truck_realizations_theta(i, :)') .* truck_realizations_y1(i, :)' - cos(truck_realizations_theta(i, :)') .* truck_realizations_y2(i, :)' + 1 / 2 * ADV_WIDTH)];
            end
            mean_ineq_coeff{i, j} = 1 / N_s * sum(d);
            temp_sum = zeros(3, 3);
            for k = 1:N_s
                temp_sum = temp_sum + (d(k, :) - mean_ineq_coeff{i, j})' * (d(k, :) - mean_ineq_coeff{i, j});
            end
            var_ineq_coeff{i, j} = 1 / N_s * temp_sum;
        end
    end
    paramOpt.coeff_realizations = coeff_realizations;
    paramOpt.mean_ineq_coeff = mean_ineq_coeff;
    paramOpt.var_ineq_coeff = var_ineq_coeff;

    % Concentration bounds
    r1 = zeros(HORIZON, 4);
    r2 = zeros(HORIZON, 4);
    beta_r1 = beta / 2;
    beta_r2 = beta / (2 * 3);
    for i = 2:HORIZON + 1
        for j = 1:4
            % mean
            lambda = eig(inv(var_ineq_coeff{i, j}));
            Tval = (3 * (N_s - 1)) / (N_s - 3) * fpdf(1-beta_r1, 3, N_s-3);
            r1(i-1, j) = sqrt(Tval/N_s/min(lambda));

            % covariance
            r2_1D = zeros(3, 1);
            for k = 1:3
                r2_1D(k) = var_ineq_coeff{i, j}(k, k) * max(abs(((N_s - 1)/chi2inv(beta_r2 / 2, N_s - 1)-1)), abs(abs(((N_s - 1)/chi2inv(1 - beta_r2 / 2, N_s - 1)-1))));
            end
            temp_sum = sum(r2_1D.^2);
            for ki = 1:3
                for kj = 1:3
                    if kj == ki
                        continue
                    else
                        temp_sum = temp_sum + (sqrt((var_ineq_coeff{i, j}(ki, ki)+r2_1D(ki)) * (var_ineq_coeff{i, j}(kj, kj) + r2_1D(kj))) + abs(var_ineq_coeff{i, j}(ki, kj)))^2;
                    end
                end
            end
            r2(i-1, j) = sqrt(temp_sum);
        end
    end
    r1 = real(r1);
    paramOpt.r1 = r1;
    paramOpt.r2 = r2;

    % Risk allocation
    paramOpt.eps_single = epsilon / (HORIZON * 4);

    % References
    paramOpt.x_0 = [EGO_X0; EGO_XDOT0; EGO_Y0; EGO_YDOT0];
    paramOpt.x_r = [0; 0; 0; 0];
    paramOpt.u_r = [0; 0];
end
