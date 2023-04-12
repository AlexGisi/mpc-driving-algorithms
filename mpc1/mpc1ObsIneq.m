function dist = obsIneq(x, u, e, data, p)

safetyPadding = 0.5;

% Obstacle coords, top left to bottom bottom left.
obs = [20 4; 21 4; 21 3.5; 20 3.5];
obsBox = collisionBox(0.5, 1, 0);
obsBox.Pose = trvec2tform([20.5, 3.75, 0]);

% Colllision box for car.
carBox = collisionBox(1.6, 2.2, 0);
T = trvec2tform([x(1), x(2), 0]);
H = axang2tform([0 0 1 x(4)]);
carBox.Pose = T * H;

[collisionStatus, dist, ~] = checkCollision(obsBox, carBox);

if collisionStatus
    dist = -10;
end

dist = -dist + safetyPadding;
end