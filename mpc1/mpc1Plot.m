function mpc1Plot(info, ref, obs)
    scale = 1;

    f = figure('NumberTitle', 'off');
    xlabel('X');
    ylabel('Y');

    scatter(ref(:, 1), ref(:, 2), 100);
    hold on;
 
    plot(ref(:, 1), ref(:, 2));
    ylim([-6 6]);

    rectangle('Position', [20 3.5 1 0.5]); % lower left coords, width, height

    poses = info.Xopt;
    scatter(poses(:, 1), poses(:, 2));
    for i = 1:size(poses, 1)
        x = poses(i, 1);
        y = poses(i, 2);
        theta = poses(i, 4);

        dx = scale * cos(theta);
        dy = scale * sin(theta);
        quiver(x, y, dx, dy, 'r', 'LineWidth', 1, 'MaxHeadSize', 2);
    end
    
end

% length = 2.2;
% width = 1.2;

% box = collisionBox(length, width, 0);
% T = trvec2tform([poses(i, 1), poses(i, 2), 0]);
% H = axang2tform([0 0 1 poses(i, 4)]);
% box.Pose = T * H;
% 
% show(box);

% poses = info.Xopt;
% 
% for i = 1:size(poses, 1)
%     x = poses(i, 1);
%     y = poses(i, 2);
%     theta = poses(i, 4);
%     plotRectangle(x, y, theta, width, length);
% end
function plotRectangle(x, y, theta, width, length)
    corners = [length/2, -length/2, -length/2, length/2;
               width/2, width/2, -width/2, -width/2];
    rotationMatrix = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    rotatedCorners = rotationMatrix * corners;
    xCorners = rotatedCorners(1, :) + x;
    yCorners = rotatedCorners(2, :) + y;
    patch(xCorners, yCorners, 'b', 'FaceAlpha', 0.2);
end