function dists = obsIneq(x, u, e, data, p)

safetyPadding = 0.5;

% Obstacle coords, top left to bottom bottom left.
obs = [20 4; 21 4; 21 3.5; 20 3.5];
obsBox = collisionBox(0.5, 1, 0);
obsBoxPose = trvec2tform([20.5 3.75 0]);
obsBox.Pose = obsBoxPose;
% show(obsBox)

dists = zeros(size(x, 1), 1);
for i=1:size(x, 1)
    xi = x(i, :);

    % Colllision box for car.
    carBox = collisionBox(1.6, 2.2, 0);
    T = trvec2tform([xi(1), xi(2), 0]);
    H = axang2tform([0 0 1 xi(4)]);
    carBoxPose = T * H;
    carBox.Pose = carBoxPose;
    % show(carBox)
    
    [collisionStatus, dist, ~] = checkCollision(obsBox, carBox);
    
    if collisionStatus
        dist = 10;
    end
    dists(i) = -dist + safetyPadding
end
end