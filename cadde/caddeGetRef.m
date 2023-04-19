% x : kinematic bicycle initial state vector.
% ref : T x 4 matrix
function [x0, u0, ref] = caddeGetRef(p, Ts)
    x0 = [0; 0; 2; pi/4];
    u0 = [0; 0];

    Xs = zeros(p, 1);
    Ys = zeros(p, 1);
    Vs = x0(3) * ones(p, 1);
    Ps = zeros(p, 1);

    Xs(1) = x0(1);
    Ys(1) = x0(2);
    Ps(1) = x0(4);

    % Assume constant velocity, heading over the reference trajectory. 
    % The velocity times the discretization time is the desired distance
    % between waypoints. To project this to the X-axis, use cos of car's
    % heading.
    trueDist = 0.5;
    addX = trueDist * cos(x0(4));
    for i=2:p
        Xs(i) = Xs(i-1) + addX;
    end

    for i=2:p
        Ys(i) = getRefY(Xs(i));
        % Ps(i) = atan(getDRef(Xs(i)));
    end

    ref = [Xs, Ys, Vs, Ps];
end

function refY = getRefY(X)
    refY = 4 * sin((2*pi / 40) * X);
end

function dRefY = getDRef(X)
    dRefY = (8 * pi / 100) * cos((2 * pi / 100) * X);
end
