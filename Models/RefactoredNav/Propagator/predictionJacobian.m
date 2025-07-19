function phi = predictionJacobian(x, u, kfInds, kfErrInds, kfConsts, time)
    N = kfErrInds.maxStateIndex;
    F = zeros(N, N);

    % === Unpack ===
    q  = x(kfInds.quat);
    bg = x(kfInds.gyroBias);
    ba = x(kfInds.accelBias);

    u_gyro  = u(1:3);
    u_accel = u(4:6);

    w_corr = u_gyro - bg;
    a_B = u_accel - ba;
    R_NB = quat2rotm(q');

    % === Attitude error (delta theta) ===
    F(kfErrInds.theta, kfErrInds.theta) = -0.5 * skew(w_corr);
    F(kfErrInds.theta, kfErrInds.gyroBias) = 0.5 * eye(3);

    % === Position ===
    F(kfErrInds.pos, kfErrInds.vel) = eye(3);

    % === Velocity ===
    F(kfErrInds.vel, kfErrInds.theta) = -R_NB * skew(a_B);
    F(kfErrInds.vel, kfErrInds.accelBias) = -R_NB;

    % === Discretization ===
    phi = expm(F * time.navDt);
end
