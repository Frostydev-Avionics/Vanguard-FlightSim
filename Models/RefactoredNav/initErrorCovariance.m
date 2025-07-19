function P = initErrorCovariance(errInds, kfConsts)

    N = errInds.maxStateIndex;
    P = zeros(N, N);

    % === Small-Angle Attitude Error (δθ) ===
    P(sub2ind(size(P), errInds.theta, errInds.theta)) = (deg2rad(5))^2;

    % === Position Error ===
    P(sub2ind(size(P), errInds.pos, errInds.pos)) = (1)^2;

    % === Velocity Error ===
    P(sub2ind(size(P), errInds.vel, errInds.vel)) = (0.5)^2;

    % === Angular Velocity Error ===
    % P(errInds.angVel, errInds.angVel) = (0.1)^2;

    % === Gyro Bias Error ===
    P(sub2ind(size(P), errInds.gyroBias, errInds.gyroBias)) = (deg2rad(0.5))^2;

    % === Accelerometer Bias Error ===
    P(errInds.accelBias(1:2), errInds.accelBias(1:2)) = (0.2)^2;
    P(errInds.accelBias(3),   errInds.accelBias(3))   = (0.5)^2;

    % === Magnetometer Bias Error ===
    P(sub2ind(size(P), errInds.magBias, errInds.magBias)) = (30e-6)^2;  % 30 uT uncertainty

    % === Inertia Error ===
    % P(sub2ind(size(P), errInds.inertia, errInds.inertia)) = (5e-8)^2;

end
