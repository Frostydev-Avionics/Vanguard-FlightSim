function kfParams = initNavParams(kins)
    kfInds   = getKfInds();
    kfErrInds = getErrorKfInds();
    kfConsts = getKfConsts();

    % State vector length
    N = kfInds.maxStateIndex;

    % Initialize struct fields
    kfParams.x        = zeros(N, 1);

    kfParams.x(kfInds.quat) = [1; 0; 0; 0];

    % Initial Guess at Inertia
    % kfParams.x(kfInds.inertia) = [kins.I_x_empty; kins.I_y_empty; kins.I_z_empty];
    
    kfParams.x_pred   = kfParams.x;
    kfParams.Q_k      = initProcessNoise(kfErrInds, kfConsts);     % [N_err x N_err]
    kfParams.P        = initErrorCovariance(kfErrInds, kfConsts);  % [N_err x N_err]
    kfParams.P_min    = kfParams.P;

    kfParams.f_pred   = zeros(N,1);
end
