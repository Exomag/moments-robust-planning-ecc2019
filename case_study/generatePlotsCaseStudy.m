function generatePlotsCaseStudy()
    % Parameters
    params = loadParameters();
    HORIZON = params.HORIZON;
    LANE_WIDTH = params.LANE_WIDTH;
    CAR_LENGTH = params.CAR_LENGTH;
    CAR_WIDTH = params.CAR_WIDTH;
    ADV_LENGTH = params.ADV_LENGTH;
    ADV_WIDTH = params.ADV_WIDTH;

    % Data
    N_trials = 100;
    costs = zeros(N_trials, 3);
    emp_viol = zeros(N_trials, 3);
    t_solver = zeros(N_trials, 3);
    problems = zeros(N_trials, 3);
    solEgoState = cell(N_trials, 3);
    solInput = cell(N_trials, 3);
    solAdvState = cell(N_trials, 3);

    % Trials
    for trial = 1:N_trials
        % Initialize
        rng(trial)

        % MRA (N_s = 5000)
        params.N_s = 5000;
        paramOpt = calcOptimizationParameters(params);
        [solData, solution, objective] = solveProblemMRA(params, paramOpt);
        if solution.problem == 0
            Pviol_joint = calcMonteCarloViolationProbability(params, solData);
        else
            Pviol_joint = NaN;
        end
        costs(trial, 1) = value(objective);
        emp_viol(trial, 1) = Pviol_joint;
        t_solver(trial, 1) = solution.solvertime;
        problems(trial, 1) = solution.problem;
        solEgoState{trial, 1} = solData.x_val;
        solInput{trial, 1} = solData.u_val;
        solAdvState{trial, 1} = [paramOpt.truck_realizations_y1(:, 1)'; paramOpt.truck_realizations_y2(:, 1)'; paramOpt.truck_realizations_theta(:, 1)'];

        % MRA (r1=r2=0)
        paramOpt.r1 = zeros(HORIZON, 4);
        paramOpt.r2 = zeros(HORIZON, 4);
        [solData, solution, objective] = solveProblemMRA(params, paramOpt);
        if solution.problem == 0
            Pviol_joint = calcMonteCarloViolationProbability(params, solData);
        else
            Pviol_joint = NaN;
        end
        costs(trial, 2) = value(objective);
        emp_viol(trial, 2) = Pviol_joint;
        t_solver(trial, 2) = solution.solvertime;
        problems(trial, 2) = solution.problem;
        solEgoState{trial, 2} = solData.x_val;
        solInput{trial, 2} = solData.u_val;
        solAdvState{trial, 2} = [paramOpt.truck_realizations_y1(:, 1)'; paramOpt.truck_realizations_y2(:, 1)'; paramOpt.truck_realizations_theta(:, 1)'];

        % MRA (N_s = 500)
        params.N_s = 500;
        rng(trial)
        paramOpt = calcOptimizationParameters(params);
        [solData, solution, objective] = solveProblemMRA(params, paramOpt);
        if solution.problem == 0
            Pviol_joint = calcMonteCarloViolationProbability(params, solData);
        else
            Pviol_joint = NaN;
        end
        costs(trial, 3) = value(objective);
        emp_viol(trial, 3) = Pviol_joint;
        t_solver(trial, 3) = solution.solvertime;
        problems(trial, 3) = solution.problem;
        solEgoState{trial, 3} = solData.x_val;
        solInput{trial, 3} = solData.u_val;
        solAdvState{trial, 3} = [paramOpt.truck_realizations_y1(:, 1)'; paramOpt.truck_realizations_y2(:, 1)'; paramOpt.truck_realizations_theta(:, 1)'];
    end

    % Figure - Costs
    viols_boxplot = -[costs(problems(:, 2) == 0, 2); costs(problems(:, 1) == 0, 1); costs(problems(:, 3) == 0, 3)] + 5;
    groups_boxplot = [zeros(sum(problems(:, 1) == 0), 1); ones(sum(problems(:, 2) == 0), 1); 2 * ones(sum(problems(:, 2) == 0), 1)];
    h1 = figure();
    hold on;
    boxplot(viols_boxplot, groups_boxplot, 'Labels', {'A', 'B', 'C'}, 'Whisker', Inf)
    ylabel('terminal position $$(\si{\meter})$$', 'interpreter', 'latex')
    set(gca, 'TickLabelInterpreter', 'latex')
    legend('hide')
    MagInset(h1, -1, [1.75, 3.25, 18.4, 20.2], [1.9, 3.4, 40, 75], {'NW', 'SW'; 'NE', 'SE'});
    legend('hide');
    set(gca, 'TickLabelInterpreter', 'latex')
    save2tikz('plots/TurningTruck_CostsBoxplot')

    % Figure - Empirical violations
    viols_boxplot = 100 * [emp_viol(problems(:, 2) == 0, 2); emp_viol(problems(:, 1) == 0, 1); emp_viol(problems(:, 3) == 0, 3)];
    groups_boxplot = [zeros(sum(problems(:, 1) == 0), 1); ones(sum(problems(:, 2) == 0), 1); 2 * ones(sum(problems(:, 2) == 0), 1)];
    figure();
    hold on;
    boxplot(viols_boxplot, groups_boxplot, 'Labels', {'A', 'B', 'C'}, 'Whisker', Inf)
    ylabel('violation probability $$(\si{\percent})$$', 'interpreter', 'latex')
    set(gca, 'TickLabelInterpreter', 'latex')
    legend('hide')
    save2tikz('plots/TurningTruck_ViolsBoxplot')

    % Figure - Frames
    ind_case = 1;
    ind_frames = [3, 5, 7, 10];
    N_frames = length(ind_frames);
    xlim_vec = repmat([-7, 45], length(ind_frames), 1);
    ylim_vec = repmat([-10, 5], length(ind_frames), 1);
    figure()
    for i = 1:N_frames
        ind = ind_frames(i);
        subplot(N_frames, 1, i);
        hold on;
        grid on
        plot(solEgoState{ind_case, 2}(1, 1:ind), solEgoState{ind_case, 2}(3, 1:ind), '-s', 'Color', [0, 100 / 255, 0], 'LineWidth', 0.25)
        plot(solEgoState{ind_case, 1}(1, 1:ind), solEgoState{ind_case, 1}(3, 1:ind), 'b-o', 'LineWidth', 0.25)
        plot(solEgoState{ind_case, 3}(1, 1:ind), solEgoState{ind_case, 3}(3, 1:ind), 'm-x', 'LineWidth', 0.25)
        rectangle('Position', [solEgoState{ind_case, 1}(1, ind) - CAR_LENGTH / 2, solEgoState{ind_case, 1}(3, ind) - CAR_WIDTH / 2, CAR_LENGTH, CAR_WIDTH], 'EdgeColor', 'b', 'FaceColor', 'none', 'LineWidth', 0.25)
        rectangle('Position', [solEgoState{ind_case, 2}(1, ind) - CAR_LENGTH / 2, solEgoState{ind_case, 2}(3, ind) - CAR_WIDTH / 2, CAR_LENGTH, CAR_WIDTH], 'EdgeColor', [0, 100 / 255, 0], 'FaceColor', 'none', 'LineWidth', 0.25)
        rectangle('Position', [solEgoState{ind_case, 3}(1, ind) - CAR_LENGTH / 2, solEgoState{ind_case, 3}(3, ind) - CAR_WIDTH / 2, CAR_LENGTH, CAR_WIDTH], 'EdgeColor', 'm', 'FaceColor', 'none', 'LineWidth', 0.25)
        plot(solAdvState{ind_case, 1}(1, 1:ind), solAdvState{ind_case, 1}(2, 1:ind), 'r-', 'LineWidth', 0.25)
        adv_edges = [ADV_LENGTH / 2 * cos(solAdvState{ind_case, 1}(3, ind)) - ADV_WIDTH / 2 * sin(solAdvState{ind_case, 1}(3, ind)); - ADV_LENGTH / 2 * sin(solAdvState{ind_case, 1}(3, ind)) - ADV_WIDTH / 2 * cos(solAdvState{ind_case, 1}(3, ind))];
        adv_edges = [adv_edges, [ADV_LENGTH / 2 * cos(solAdvState{ind_case, 1}(3, ind)) + ADV_WIDTH / 2 * sin(solAdvState{ind_case, 1}(3, ind)); - ADV_LENGTH / 2 * sin(solAdvState{ind_case, 1}(3, ind)) + ADV_WIDTH / 2 * cos(solAdvState{ind_case, 1}(3, ind))]];
        adv_edges = [adv_edges, -adv_edges, adv_edges(:, 1)];
        for j = 1:4
            plot(solAdvState{ind_case, 1}(1, ind)+adv_edges(1, :), solAdvState{ind_case, 1}(2, ind)+adv_edges(2, :), 'r', 'LineWidth', 0.25)
        end
        plot(xlim_vec, LANE_WIDTH*ones(2, 1), '-k', 'LineWidth', 2)
        plot([xlim_vec(1), 21], -LANE_WIDTH*ones(2, 1), '-k', 'LineWidth', 2)
        plot([35, xlim_vec(end)], -LANE_WIDTH*ones(2, 1), '-k', 'LineWidth', 2)
        plot(21*ones(2, 1), [ylim_vec(end, 1), -LANE_WIDTH], '-k', 'LineWidth', 2)
        plot(35*ones(2, 1), [ylim_vec(end, 1), -LANE_WIDTH], '-k', 'LineWidth', 2)
        plot(xlim_vec, 0*ones(2, 1), '--k', 'LineWidth', 1)
        xlim(xlim_vec(i, :))
        ylim(ylim_vec(i, :))
        xticks(-5:10:45)
        if i ~= N_frames
            set(gca, 'xticklabel', [])
        else
            xticklabels({'0', '10', '20', '30', '40', '50'})
        end
        ylabel('$$x_2 (\si{\meter})$$', 'interpreter', 'latex')
        if i ~= N_frames
            legend('hide')
        else
            legend({'A', 'B', 'C'}, 'Location', 'SouthWest', 'Orientation', 'horizontal', 'interpreter', 'latex');
        end
        set(gca, 'TickLabelInterpreter', 'latex')
    end
    xlabel('$$x_1 (\si{\meter})$$', 'interpreter', 'latex')
    save2tikz('plots/TurningTruck_Frames')

    % Figure - Monte Carlo
    sampleData = sampleTruckTrajectories(params);
    minAdvX = sampleData.minAdvX;
    maxAdvX = sampleData.maxAdvX;
    xlim_vec = [-10, 75];
    ylim_vec = [-12, 5];
    h2 = figure();
    hold on;
    grid on
    plot(solEgoState{ind_case, 2}(1, :), solEgoState{ind_case, 2}(3, :), '-s', 'Color', [0, 100 / 255, 0], 'LineWidth', 0.25)
    plot(solEgoState{ind_case, 1}(1, :), solEgoState{ind_case, 1}(3, :), 'b-o', 'LineWidth', 0.25)
    plot(solEgoState{ind_case, 3}(1, :), solEgoState{ind_case, 3}(3, :), 'm-x', 'LineWidth', 0.25)
    rectangle('Position', [solEgoState{ind_case, 2}(1, 1) - CAR_LENGTH / 2, solEgoState{ind_case, 2}(3, 1) - CAR_WIDTH / 2, CAR_LENGTH, CAR_WIDTH], 'EdgeColor', 'k', 'FaceColor', 'none', 'LineWidth', 0.25)
    patch([minAdvX; flipud(maxAdvX)], [solAdvState{ind_case, 1}(2, :)'; flipud(solAdvState{ind_case, 1}(2, :)')], 'r-', 'FaceAlpha', 0.25, 'EdgeColor', 'r')
    rectangle('Position', [solAdvState{ind_case, 1}(1, 1) - ADV_LENGTH / 2, solAdvState{ind_case, 1}(2, 1) - ADV_WIDTH / 2, ADV_LENGTH, ADV_WIDTH], 'EdgeColor', 'r', 'FaceColor', 'none', 'LineWidth', 0.25)
    plot(xlim_vec, LANE_WIDTH*ones(2, 1), '-k', 'LineWidth', 2)
    plot([xlim_vec(1), 21], -LANE_WIDTH*ones(2, 1), '-k', 'LineWidth', 2)
    plot([35, xlim_vec(end)], -LANE_WIDTH*ones(2, 1), '-k', 'LineWidth', 2)
    plot(21*ones(2, 1), [ylim_vec(end, 1), -LANE_WIDTH], '-k', 'LineWidth', 2)
    plot(35*ones(2, 1), [ylim_vec(end, 1), -LANE_WIDTH], '-k', 'LineWidth', 2)
    plot(xlim_vec, 0*ones(2, 1), '--k', 'LineWidth', 1)
    xlim(xlim_vec)
    ylim(ylim_vec)
    xticks(-5:20:75)
    xticklabels({'0', '20', '40', '60', '80'})
    xlabel('$$x_1 (\si{\meter})$$', 'interpreter', 'latex')
    ylabel('$$x_2 (\si{\meter})$$', 'interpreter', 'latex')
    legend('hide')
    set(gca, 'TickLabelInterpreter', 'latex')
    MagInset(h2, -1, [-1, 16, -2.4, -0.6], [-7.5, 18.5, -11, -6], {'SW', 'NW'; 'SE', 'NE'});
    grid off;
    legend('hide');
    set(gca, 'xtick', []);
    set(gca, 'ytick', []);
    set(gca, 'XTickLabel', []);
    set(gca, 'YTickLabel', []);
    save2tikz('plots/TurningTruck_Trajectories')
end
