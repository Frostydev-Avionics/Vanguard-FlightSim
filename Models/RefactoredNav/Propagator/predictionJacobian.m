function phi = predictionJacobian(x, u, kfInds, kfErrInds, kfConsts, time)

    % N = max(structfun(@(v) max(v(:)), errInds));  % Size of error state
    N = kfErrInds.maxStateIndex;
    F = zeros(N, N);

    %% State Unpacking (from nominal state)
    q = x(kfInds.quat);
    p = x(kfInds.pos);
    v = x(kfInds.vel);
    w = x(kfInds.angVel);
    bg = x(kfInds.gyroBias);
    ba = x(kfInds.accelBias);
    bm = x(kfInds.magBias);
    I  = x(kfInds.inertia);

    %% Inputs
    u_gyro  = u(1:3);
    u_accel = u(4:6);
    w_ib_b = u_gyro - bg;
    a_B = u_accel - ba;
    R_TB = quat2rotm(q');  % Transpose: body → world
    a_T = R_TB * a_B;

    %% === ATTITUDE ERROR (δθ) DYNAMICS ===
    % del_theta_dot = -0.5 * skew(w_ib_b) * del_theta - 0.5 * del_w_b
    F(kfErrInds.theta, kfErrInds.theta) = -0.5 * skew(w_ib_b);
    F(kfErrInds.theta, kfErrInds.gyroBias) = -0.5 * eye(3);

    %% === POSITION ===
    F(kfErrInds.pos, kfErrInds.vel) = eye(3);

    %% === VELOCITY ===
    F(kfErrInds.vel, kfErrInds.theta) = -R_TB * skew(a_B);
    F(kfErrInds.vel, kfErrInds.accelBias) = -R_TB;

    %% === ANGULAR VELOCITY (dw/dt) ===
    I_mat = diag(I);
    dw_dw = -I_mat \ (skew(I_mat * w) + skew(w) * I_mat);
    F(kfErrInds.angVel, kfErrInds.angVel) = dw_dw;

    % d(del_w)/d(del_I)
    dI = 1e-6;
    for i = 1:3
        I_pert = I;
        I_pert(i) = I_pert(i) + dI;
        I_mat_pert = diag(I_pert);

        dw_pert = I_mat_pert \ (-cross(w, I_mat_pert * w));
        dw_nom = I_mat \ (-cross(w, I_mat * w));
        F(kfErrInds.angVel, kfErrInds.inertia(i)) = (dw_pert - dw_nom) / dI;
    end

    %% === DISCRETIZATION ===
    phi = expm(F * time.navDt);
end
