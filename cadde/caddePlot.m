function caddePlot(info, ref)
    f = figure('NumberTitle', 'off');
    xlabel('X');
    ylabel('Y');

    scatter(ref(:, 1), ref(:, 2), 100);
    hold on;
    % plot(ref(:, 1), ref(:, 2));
    ylim([-6 6]);

    scatter(info.Xopt(:, 1), info.Xopt(:, 2));
end