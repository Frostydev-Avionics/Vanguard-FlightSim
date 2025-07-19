function f = predictionFunction(x, u, kfInds)

    % === Force column vectors ===
    x = x(:);
    u = u(:);

    % === Unpack state ===
    q  = x(kfInds.quat);        % Quaternion: Body-to-NED [w x y z]
    p  = x(kfInds.pos);         % Position in NED
    v  = x(kfInds.vel);         % Velocity in NED
    w  = x(kfInds.angVel);      % Angular velocity in body frame
    bg = x(kfInds.gyroBias);    % Gyroscope bias
    ba = x(kfInds.accelBias);   % Accelerometer bias
    bm = x(kfInds.magBias);     % Magnetometer bias (unused here)
    I  = x(kfInds.inertia);     % Inertia (diagonal)

    % === Inputs ===
    u_gyro  = u(1:3);
    u_accel = u(4:6);

    % === Bias-corrected angular velocity ===
    w_ib_b = u_gyro - bg;

    % === Quaternion derivative ===
    % dq/dt = 0.5 * Omega(w) * q
    Omega = [0, -w_ib_b';
             w_ib_b, -skew(w_ib_b)];
    dq = 0.5 * Omega * q;

    % === Position derivative ===
    dp = v;

    % === Velocity derivative ===
    gravity = [0; 0; 9.80665];  % Gravity in NED
    a_B = u_accel - ba;        % Corrected body accel
    R_NB = quat2rotm(q');      % Body-to-NED

    dv = R_NB * a_B + gravity;

    % === Angular acceleration (rigid body dynamics) ===
    I_mat = diag(I);
    dw = I_mat \ (-cross(w, I_mat * w));  % dw/dt = I⁻¹(-w×(I·w))

    % === No dynamics for biases / inertia ===
    dbg = zeros(3,1);
    dba = zeros(3,1);
    dbm = zeros(3,1);
    dI  = zeros(3,1);

    % === Stack state derivatives ===
    f = [dq;
         dp;
         dv;
         dw;
         dbg;
         dba;
         dbm;
         dI];
end
