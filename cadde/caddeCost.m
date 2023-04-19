% Where p the prediction horizon:
% 
% x : (p+1) x Nx array, where x(1, :) is the current state.
% u : (p+1) x Nu array
% e : slack variable
% data.References: p x Ny array
% 
% https://www.mathworks.com/help/mpc/ug/specify-cost-function-for-nonlinear-mpc.html 
%
function cost = caddeCost(x, u, ~, data, ~)

cost = 0;

Q = diag([20, 20, 0.1, 0]);
R = diag([0.1, 0.1]);

for i = 1:data.PredictionHorizon
    refDiff = (x(i, :) - data.References(i, :))';
    cost = cost + refDiff' * Q * refDiff;

    ui = u(i, :)';
    cost = cost + ui' * R * ui;

    % TODO: add du's
end

end