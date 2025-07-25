function Q_k = initProcessNoise(errInds, kfConsts)

    N = errInds.maxStateIndex;
    Q_k = zeros(N, N);

    % === Attitude Noise ===
    Q_k(sub2ind(size(Q_k), errInds.theta, errInds.theta)) = (deg2rad(1))^2;

    % === Position Noise ===
    Q_k(sub2ind(size(Q_k), errInds.pos, errInds.pos)) = (3e-3)^2;

    % === Angular Velocity Noise ===
    % Q_k(errInds.angVel, errInds.angVel) = (1e-3)^2;

    % === Velocity Noise ===
    Q_k(sub2ind(size(Q_k), errInds.vel, errInds.vel)) = (3e-5)^2;

    % === Gyroscope Bias Random Walk ===
    Q_k(sub2ind(size(Q_k), errInds.gyroBias, errInds.gyroBias)) = (deg2rad(1e-2))^2;

    % === Accelerometer Bias Random Walk ===
    Q_k(sub2ind(size(Q_k), errInds.accelBias, errInds.accelBias)) = (1e-2)^2;

    % === Magnetometer Bias Random Walk ===
    Q_k(sub2ind(size(Q_k), errInds.magBias, errInds.magBias)) = (1e-7)^2;

    % === Gravity Noise ===
    Q_k(sub2ind(size(Q_k), errInds.g, errInds.g)) = (1e-5)^2;

    % === Inertia Noise ===
    % Q_k(sub2ind(size(Q_k), errInds.inertia, errInds.inertia)) = (1e-10)^2;

end
