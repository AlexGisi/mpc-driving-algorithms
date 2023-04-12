% Forward Euler discretization of Eq. 5 in Carvalho et. al. (2015).
% x(1) : Global position along the X axis.
% x(2) : Global position along the Y axis.
% x(3) : Velocity along the vehicle-aligned x-axis.
% x(4) : Velocity alone the vehicle-aligned y-axis.
% x(5) : Angle from the X axis.
% x(6) : First derivative of x(5).
% u(1) : Acceleration.
% u(2) : Steering angle.
function z = dynamicBicycle(x, u, p)
    dt = p(1);
    
    m       = 550;      % Total mass of vehicle                          (kg)
    Iz      = 960;      % Yaw moment of inertia of vehicle               (m*N*s^2)
    lf      = 1;        % Longitudinal distance from c.g. to front tires (m)
    lr      = 1.2;      % Longitudinal distance from c.g. to rear tires  (m)
    Cf      = 19000;    % Cornering stiffness of front tires             (N/rad)
    Cr      = 33000;    % Cornering stiffness of rear tires              (N/rad)

    % Front and rear slip angles.
    alpha_f = -u(2) * atan((x(4) + lf*x(6)) / x(3));
    alpha_r = atan((x(4) + lr*x(6)) / x(3));

    % Front and rear lateral forces.
    F_cf = alpha_f * Cf;
    F_cr = alpha_r * Cr;

    z(1) = x(1) + dt * (x(3)*cos(x(5)) - x(4)*sin(x(5)));
    z(2) = x(2) + dt * (x(3)*sin(x(5)) + x(4)*cos(x(5)));
    z(3) = x(3) + dt * (x(6) * x(4) + u(1));
    z(4) = x(4) + dt * (-x(6)*x(4) + (2/m)*(F_cf * cos(delta) + F_cr));
    z(5) = x(5) + dt * x(6);
    z(6) = x(6) + (2 / Iz)*(lf*F_cf - lr*F_cr);
end
