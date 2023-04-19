% One level MPC for obstacle avoidance and path tracking based on dynamic bicycle model.

function [mpcinfo] = cadde(x0, u0, ref)
    %% Set up NMPC problem.
    Hp = 20;
    Hc = 20;
    Ts = 0.2;

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

    nlobj.Optimization.CustomCostFcn = "caddeCost";

    nlobj.MV(1).Name = "a";
    nlobj.MV(2).Name = "s";

    nlobj.MV(1).Min = -1;
    nlobj.MV(1).Max = 1;
    nlobj.MV(2).Min = -pi/6;
    nlobj.MV(2).Max = pi/6;

    % nlobj.Optimization.CustomIneqConFcn = "mpc1ObsIneq";
    
    validateFcns(nlobj, x0, u0, [], {Ts});
    
    opt = nlmpcmoveopt();
    opt.Parameters = {Ts};
    opt.X0 = ref;

    fprintf('NMPC is running...\n');
    tic;[~,~,info] = nlmpcmove(nlobj, x0, u0, ref, [], opt); t=toc;
    fprintf('Calculation Time = %s; Objective cost = %s; ExitFlag = %s; Iterations = %s\n',...
    num2str(t),num2str(info.Cost),num2str(info.ExitFlag),num2str(info.Iterations));

    mpcinfo = info;
end
