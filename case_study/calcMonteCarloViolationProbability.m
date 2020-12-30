function Pviol_joint = calcMonteCarloViolationProbability(params, solData)
    % Parameters
    SAMPLING_TIME = params.SAMPLING_TIME;
    HORIZON = params.HORIZON;
    CAR_LENGTH = params.CAR_LENGTH;
    CAR_WIDTH = params.CAR_WIDTH;
    ADV_LENGTH = params.ADV_LENGTH;
    ADV_WIDTH = params.ADV_WIDTH;
    ADV_X0 = params.ADV_X0;
    ADV_Y0 = params.ADV_Y0;
    ADV_VEL0 = params.ADV_VEL0;
    ADV_THETA0 = params.ADV_THETA0;

    % Solution
    x_val = solData.x_val;

    % Initialization
    Nviol = 0;
    rng(0)

    % Realizations
    N_MonteCarlo = 1e5;
    w_realizations = cell(1, N_MonteCarlo);
    for k = 1:N_MonteCarlo
        w_realizations{k}.theta = zeros(HORIZON+1, 1);
        for i = 1:HORIZON
            w_realizations{k}.theta(i) = pi / 2 / (HORIZON + 1) - 0.03 + min(0.06, pi/2-sum(w_realizations{k}.theta)-pi/2/(HORIZON + 1)+0.03) * rand;
        end
        w_realizations{k}.theta(HORIZON+1) = pi / 2 - sum(w_realizations{k}.theta);
        w_realizations{k}.v = -ADV_VEL0 * ones(HORIZON+1, 1);
    end

    % Calculate violation probability via Monte Carlo sampling
    for i = 1:N_MonteCarlo
        truck_realization = generateTruckTrajectory([ADV_X0; ADV_Y0; ADV_THETA0], SAMPLING_TIME, w_realizations{i});
        truck_realization = truck_realization';
        truck_realization(3, :) = -truck_realization(3, :);
        found_collision = false;
        for j = 2:HORIZON + 1
            for k = 1:5
                if k == 1
                    temp_x = [x_val(1, j) + CAR_LENGTH / 2, x_val(3, j) + CAR_WIDTH / 2];
                elseif k == 2
                    temp_x = [x_val(1, j) + CAR_LENGTH / 2, x_val(3, j) - CAR_WIDTH / 2];
                elseif k == 3
                    temp_x = [x_val(1, j) - CAR_LENGTH / 2, x_val(3, j) + CAR_WIDTH / 2];
                elseif k == 4
                    temp_x = [x_val(1, j) - CAR_LENGTH / 2, x_val(3, j) - CAR_WIDTH / 2];
                elseif k == 5
                    temp_x = [x_val(1, j), x_val(3, j)];
                end
                if cos(truck_realization(3, j)) * temp_x(1) - sin(truck_realization(3, j)) * temp_x(2) <= cos(truck_realization(3, j)) * truck_realization(1, j) - sin(truck_realization(3, j)) * truck_realization(2, j) + 1 / 2 * ADV_LENGTH ...
                        && sin(truck_realization(3, j)) * temp_x(1) + cos(truck_realization(3, j)) * temp_x(2) <= sin(truck_realization(3, j)) * truck_realization(1, j) + cos(truck_realization(3, j)) * truck_realization(2, j) + 1 / 2 * ADV_WIDTH ...
                        && -cos(truck_realization(3, j)) * temp_x(1) + sin(truck_realization(3, j)) * temp_x(2) <= -cos(truck_realization(3, j)) * truck_realization(1, j) + sin(truck_realization(3, j)) * truck_realization(2, j) + 1 / 2 * ADV_LENGTH ...
                        && -sin(truck_realization(3, j)) * temp_x(1) - cos(truck_realization(3, j)) * temp_x(2) <= -sin(truck_realization(3, j)) * truck_realization(1, j) - cos(truck_realization(3, j)) * truck_realization(2, j) + 1 / 2 * ADV_WIDTH
                    found_collision = true;
                    break;
                end
            end
            if found_collision
                Nviol = Nviol + 1;
                break
            end
        end
    end
    Pviol_joint = Nviol / N_MonteCarlo;
end
