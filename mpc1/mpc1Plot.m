function mpc1Plot(info, ref, obs)
    f = figure('NumberTitle', 'off');
    xlabel('X');
    ylabel('Y');

    scatter(ref(:, 1), ref(:, 2), 100);
    hold on;
    plot(ref(:, 1), ref(:, 2));
    ylim([-6 6]);
    rectangle('Position', [20 3.5 1 0.5]); % lower left coords, width, height

    scatter(info.Xopt(:, 1), info.Xopt(:, 2));
end