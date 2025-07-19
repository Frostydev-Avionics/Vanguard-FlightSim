function plotNav(out, kfInds, kfErrInds)
    close all; clc;
    
    ICM20948_PARAMS = getICM20948Params();
    MMC5983_PARAMS  = getMMC5983Params();
    
    % === Extract Data ===
    truthTime = out.tout;
    pos_T_true = out.P_T.Data;
    RPY = rad2deg(out.RPY.Data);

    N = size(out.R_BT.Data, 3);
    q_true = zeros(4, N);  % [4 x N]
    vel_T_true = zeros(3,N);
    for i = 1:N
        R = out.R_BT.Data(:,:,i);  % [3x3]
        q_true(:,i) = rotm2quat(R');  % Transpose from R_BT to R_TB
        vel_T_true(:, i) = R' * out.V_B.Data(i, :)';  % Transform velocity to the true frame
    end

    % Navigation state estimates
    navTime = out.NavBus.x.Time;
    x_est = out.NavBus.x.Data;
    P     = out.NavBus.P.Data;
    
    q_est  = x_est(kfInds.quat, :);
    gb_est = x_est(kfInds.gyroBias, :);
    ab_est = x_est(kfInds.accelBias, :);
    mb_est = x_est(kfInds.magBias, :);
    pos_est = x_est(kfInds.pos, :);
    vel_est = x_est(kfInds.vel, :);
    
    % === Resample Ground Truth ===
    pos_true_resampled = resampleTimeSeries(pos_T_true, truthTime, navTime);
    vel_true_resampled = resampleTimeSeries(vel_T_true, truthTime, navTime);
    q_true_resampled   = resampleTimeSeries(q_true, truthTime, navTime);
    
    % === Quaternion Error as Angle ===
    q_err = zeros(length(navTime), 4);
    for i = 1:length(navTime)
        qT = q_true_resampled(:, i)';
        qE = q_est(:, i)';
        q_err(i, :) = quatmultiply(qT, quatinv(qE));
    end
    angle_error = 2 * acos(min(1, abs(q_err(:,1))));  % Angle in radians
    angle_error_deg = rad2deg(angle_error);

    figure('Name', 'Attitude Error');
    plot(navTime, angle_error_deg, 'r');
    ylabel('Angle Error (deg)');
    xlabel('Time (s)');
    title('Attitude Error Magnitude');
    grid on;

    % === Convert to ZYX Euler Angles ===
    quatToEulerZYX = @(q) rad2deg(quat2eul(q', 'ZYX'));  % returns [yaw pitch roll] rows
    
    eul_true = quatToEulerZYX(q_true_resampled);  % [N x 3]
    eul_est  = quatToEulerZYX(q_est);             % [N x 3]
    eul_error = wrapTo180(eul_true - eul_est);    % error in deg
    
    % === Plot Euler Angle Estimates ===
    figure('Name', 'Euler Angle Estimates');
    labels = {'Yaw (deg)', 'Pitch (deg)', 'Roll (deg)'};
    for i = 1:3
        subplot(3,1,i);
        plot(navTime, eul_true(:,i), 'k', 'DisplayName', 'True'); hold on;
        plot(navTime, eul_est(:,i), 'r', 'DisplayName', 'Estimated');
        ylabel(labels{i});
        grid on;
        legend();
    end
    xlabel('Time (s)');
    sgtitle('True vs Estimated Euler Angles (ZYX)');
    linkaxes(findall(gcf, 'Type', 'axes'), 'x');

    % === State Errors ===
    pos_error = pos_true_resampled - pos_est;
    vel_error = vel_true_resampled - vel_est;
    ab_error = ab_est - ICM20948_PARAMS.accel.bias;
    gb_error = gb_est - ICM20948_PARAMS.gyro.bias;
    mb_error = mb_est - MMC5983_PARAMS.bias;

    % === Plot Errors with Covariance ===
    plotWithCovariance(navTime, eul_error, P, kfErrInds.theta, ...
    'Euler Angle Errors (ZYX)', {'Yaw', 'Pitch', 'Roll'});
    plotWithCovariance(navTime, pos_error, P, kfErrInds.pos, 'Position Error (m)', {'X', 'Y', 'Z'});
    plotWithCovariance(navTime, vel_error, P, kfErrInds.vel, 'Velocity Error (m/s)', {'Vx', 'Vy', 'Vz'});
    plotWithCovariance(navTime, gb_error,  P, kfErrInds.gyroBias, 'Gyro Bias Estimation (rad/s)', {'X', 'Y', 'Z'});
    plotWithCovariance(navTime, ab_error,  P, kfErrInds.accelBias, 'Accel Bias Estimation (m/s²)', {'X', 'Y', 'Z'});
    plotWithCovariance(navTime, mb_error,  P, kfErrInds.magBias, 'Mag Bias Estimation (uT)', {'X', 'Y', 'Z'});
    
end


function plotWithCovariance(timeVec, errorVec, P, inds, yLabelStr, labels)
    % Ensure errorVec is [N x dim]
    if size(errorVec, 2) == length(inds)
        err = errorVec;  % already [N x dim]
    elseif size(errorVec, 1) == length(inds)
        err = errorVec';  % transpose to [N x dim]
    else
        [s1, s2] = size(errorVec); 
        error('Error vector has wrong shape. Expected [dim x N] or [N x dim], got [%d x %d]', s1, s2);
    end

    N = length(timeVec);
    dim = length(inds);
    sigma = zeros(N, dim);
    for i = 1:N
        for j = 1:dim
            sigma(i,j) = sqrt(P(inds(j), inds(j), i));
        end
    end

    figure('Name', yLabelStr);
    for j = 1:dim
        subplot(dim,1,j);
        plot(timeVec, err(:,j), 'r', 'DisplayName', 'Error'); hold on;
        plot(timeVec, sigma(:,j), 'b--', 'DisplayName', '+1\sigma');
        plot(timeVec, -sigma(:,j), 'b--', 'DisplayName', '-1\sigma');
        ylabel([labels{j}, ' ', yLabelStr]);
        grid on;
        legend();
    end
    xlabel('Time (s)');
    sgtitle([yLabelStr, ' with ±1\sigma Covariance Bounds']);

    linkaxes(findall(gcf, 'Type', 'axes'), 'x');
end

function data_resamp = resampleTimeSeries(truthData, truthTime, navTime)
    % Convert [M x 1 x N] to [M x N]
    if ndims(truthData) == 3
        data = squeeze(truthData);  % [M x N]
    else
        data = truthData;  % already [M x N]
    end

    % Resample each row independently
    M = size(data, 1);
    data_resamp = zeros(M, length(navTime));
    for i = 1:M
        data_resamp(i, :) = interp1(truthTime, data(i, :), navTime, 'linear', 'extrap');
    end
end
