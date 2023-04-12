% Forward Euler discretization of Eq. 4 in Carvalho et. al. (2015).
% x(1) : Global position along the X axis.
% x(2) : Global position along the Y axis.
% x(3) : Scalar speed.
% x(4) : Yaw angle from X axis.
% u(1) : Acceleration.
% u(2) : Steering angle.
function z = kinematicBicycle(x, u, p)
    z = zeros(4, 1);
    dt = p(1);

    m       = 550;      % Total mass of vehicle                          (kg)
    Iz      = 960;      % Yaw moment of inertia of vehicle               (m*N*s^2)
    lf      = 1;        % Longitudinal distance from c.g. to front tires (m)
    lr      = 1.2;      % Longitudinal distance from c.g. to rear tires  (m)
    Cf      = 19000;    % Cornering stiffness of front tires             (N/rad)
    Cr      = 33000;    % Cornering stiffness of rear tires              (N/rad)
    
    beta = atan((lr * tan(u(2)))/(lf + lr));  % Slip angle.

    z(1) = x(1) + dt * x(3) * cos(x(4) + beta);
    z(2) = x(2) + dt * sin(x(4) + beta);
    z(3) = x(3) + dt * u(1);
    z(4) = x(4) + dt * (x(3) / lr) * sin(beta);
end