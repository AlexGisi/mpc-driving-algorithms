% One level MPC for obstacle avoidance and path tracking based on dynamic bicycle model.

function [mpcinfo, ref, obs] = mpc1

    m       = 550;      % Total mass of vehicle                          (kg)
    Iz      = 960;      % Yaw moment of inertia of vehicle               (m*N*s^2)
    lf      = 1;        % Longitudinal distance from c.g. to front tires (m)
    lr      = 1.2;      % Longitudinal distance from c.g. to rear tires  (m)
    Cf      = 19000;    % Cornering stiffness of front tires             (N/rad)
    Cr      = 33000;    % Cornering stiffness of rear tires              (N/rad)

    % Obstacle coords, top left to bottom bottom left.
    obs = [20 4; 21 4; 21 3.5; 20 3.5];

    %% Set up NMPC problem.
    Hp = 50;
    Hc = 150;
    Ts = 0.2;

    x0 = [0; 0; 3.5; getDRef(0)];
    u0 = [0; 0];
    ref = getRef(x0, Hp, Ts);

    nx = 4;
    nu = 2;
    nlobj = nlmpc(nx, nx, nu);
    nlobj.Ts = Ts;
    nlobj.Model.NumberOfParameters = 1;  % Ts
    nlobj.PredictionHorizon = Hp;
    nlobj.ControlHorizon = Hc;

    nlobj.Model.StateFcn = "kinematicBicycle";
    nlobj.Model.IsContinuousTime = false;
    nlobj.States(1).Name = "X";
    nlobj.States(2).Name = "Y";
    nlobj.States(3).Name = "V";
    nlobj.States(4).Name = "P";

    nlobj.Optimization.CustomCostFcn = 'mpc1Cost';

    nlobj.MV(1).Name = "a";
    nlobj.MV(2).Name = "s";

    nlobj.MV(1).Min = -1;
    nlobj.MV(1).Max = 1;
    nlobj.MV(2).Min = -pi/6;
    nlobj.MV(2).Max = pi/6;

    nlobj.Optimization.CustomIneqConFcn = "mpc1ObsIneq";
    
    validateFcns(nlobj, x0, [0; 0], [], {Ts});
    
    opt = nlmpcmoveopt();
    opt.Parameters = {Ts};
    opt.X0 = ref;

    fprintf('NMPC is running...\n');
    tic;[~,~,info] = nlmpcmove(nlobj, x0, u0, ref, [], opt); t=toc;
    fprintf('Calculation Time = %s; Objective cost = %s; ExitFlag = %s; Iterations = %s\n',...
    num2str(t),num2str(info.Cost),num2str(info.ExitFlag),num2str(info.Iterations));

    mpcinfo = info;
end

function refY = getRefY(X)
    refY = 4 * sin((2*pi / 100) * X);
end

function dRefY = getDRef(X)
    dRefY = (8 * pi / 100) * cos((2 * pi / 100) * X);
end


% x : kinematic bicycle initial state vector.
% ref : T x 4 matrix
function ref = getRef(x, p, Ts)
    Xs = zeros(p, 1);
    Ys = zeros(p, 1);
    Vs = x(3) * ones(p, 1);
    Ps = zeros(p, 1);

    Xs(1) = x(1);
    Ys(1) = x(2);
    Ps(1) = x(4);

    % Assume constant velocity, heading over the reference trajectory. 
    % The velocity times the discretization time is the desired distance
    % between waypoints. To project this to the X-axis, use cos of car's
    % heading.
    trueDist = Vs(1) * Ts;
    addX = trueDist * cos(x(4));
    for i=2:p
        Xs(i) = Xs(i-1) + addX;
    end

    for i=2:p
        Ys(i) = getRefY(Xs(i));
        Ps(i) = atan(getDRef(Xs(i)));
    end

    ref = [Xs, Ys, Vs, Ps];
end
