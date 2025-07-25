function f = predictionFunction(x, u, kfInds)
    x = x(:); u = u(:);

    % === Unpack state ===
    q  = x(kfInds.quat);        % [w x y z]
    p  = x(kfInds.pos);         % Position in NED
    v  = x(kfInds.vel);         % Velocity in NED
    bg = x(kfInds.gyroBias);    % Gyro bias
    ba = x(kfInds.accelBias);   % Accel bias
    bm = x(kfInds.magBias);     % Mag bias (still unused)
    g  = x(kfInds.g);

    % === Inputs ===
    u_gyro  = u(1:3);           % Measured angular velocity
    u_accel = u(4:6);           % Measured acceleration

    % === Quaternion derivative (gyro - bias drives rotation) ===
    w_corr = u_gyro - bg;
    Omega = [0, -w_corr';
             w_corr, -skew(w_corr)];
    dq = 0.5 * Omega * q;

    % === Position and velocity ===
    dp = v;

    gravity = [0; 0; g];
    a_B = u_accel - ba;
    R_NB = quat2rotm(q');
    dv = R_NB * a_B + gravity;

    % === Biases constant ===
    dbg = zeros(3,1);
    dba = zeros(3,1);
    dbm = zeros(3,1);

    % === Gravity Change ===
    dg = 0;

    % === Output ===
    f = [dq;
         dp;
         dv;
         dbg;
         dba;
         dbm;
         dg];
end
