function y = generateTruckTrajectory(x0, Ts, w)
    y = zeros(length(w.theta), length(x0));
    y(1, :) = x0;
    for i = 2:size(y, 1)
        y(i, :) = y(i-1, :) + Ts * [w.v(i - 1) * cos(y(i - 1, 3)), w.v(i - 1) * sin(y(i - 1, 3)), 1 / Ts * w.theta(i - 1)];
    end
end
