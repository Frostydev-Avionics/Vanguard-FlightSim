function P = initErrorCovariance(errInds, kfConsts)

    N = errInds.maxStateIndex;
    P = zeros(N, N);

    % === Small-Angle Attitude Error (δθ) ===
    P(sub2ind(size(P), errInds.theta, errInds.theta)) = (deg2rad(5))^2; % [rad]

    % === Position Error ===
    P(sub2ind(size(P), errInds.pos, errInds.pos)) = (1)^2; % [m]

    % === Velocity Error ===
    P(sub2ind(size(P), errInds.vel, errInds.vel)) = (0.5)^2; % [m/s]

    % === Angular Velocity Error ===
    % P(errInds.angVel, errInds.angVel) = (0.1)^2;

    % === Gyro Bias Error ===
    P(sub2ind(size(P), errInds.gyroBias, errInds.gyroBias)) = (deg2rad(0.5))^2; % [rad/s]

    % === Accelerometer Bias Error ===
    P(errInds.accelBias(1:2), errInds.accelBias(1:2)) = (1e-3)^2; % [m/s^2]
    P(errInds.accelBias(3),   errInds.accelBias(3))   = (1e-3)^2; % [m/s^2]

    % === Magnetometer Bias Error ===
    P(sub2ind(size(P), errInds.magBias, errInds.magBias)) = (30e-6)^2;  % 30 uT uncertainty
    
    % === Gravity Error ===
    P(sub2ind(size(P), errInds.g, errInds.g)) = 1e-5^2; % [m/s^2]

    % === Inertia Error ===
    % P(sub2ind(size(P), errInds.inertia, errInds.inertia)) = (5e-8)^2;

end
