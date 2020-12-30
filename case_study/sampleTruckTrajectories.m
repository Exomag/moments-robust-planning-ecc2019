function sampleData = sampleTruckTrajectories(params)
    % Parameters
    SAMPLING_TIME = params.SAMPLING_TIME;
    HORIZON = params.HORIZON;
    ADV_X0 = params.ADV_X0;
    ADV_Y0 = params.ADV_Y0;
    ADV_VEL0 = params.ADV_VEL0;
    ADV_THETA0 = params.ADV_THETA0;

    % Initialization
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
    sampleData.w_realizations = w_realizations;

    % Monte Carlo
    minAdvX = +inf(HORIZON+1, 1);
    maxAdvX = -inf(HORIZON+1, 1);
    for i = 2:N_MonteCarlo
        truck_realization = generateTruckTrajectory([ADV_X0; ADV_Y0; ADV_THETA0], SAMPLING_TIME, w_realizations{i});
        truck_realization = truck_realization';
        truck_realization(3, :) = -truck_realization(3, :);
        for j = 1:HORIZON + 1
            if minAdvX(j) > truck_realization(1, j)
                minAdvX(j) = truck_realization(1, j);
            end
            if maxAdvX(j) < truck_realization(1, j)
                maxAdvX(j) = truck_realization(1, j);
            end
        end
    end
    sampleData.minAdvX = minAdvX;
    sampleData.maxAdvX = maxAdvX;
end
