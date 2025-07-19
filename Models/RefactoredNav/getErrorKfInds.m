function errInds = getErrorKfInds()
    idx = 1;

    % Small angle error (attitude)
    errInds.theta = idx:idx+2;       idx = idx + 3;

    % Position
    errInds.pos = idx:idx+2;         idx = idx + 3;

    % Velocity
    errInds.vel = idx:idx+2;         idx = idx + 3;

    % Gyro bias
    errInds.gyroBias = idx:idx+2;    idx = idx + 3;

    % Accel bias
    errInds.accelBias = idx:idx+2;   idx = idx + 3;

    % Mag bias
    errInds.magBias = idx:idx+2;     idx = idx + 3;

    % Compute max index from all fields
    allIndices = struct2cell(errInds);
    flatIndices = cellfun(@(x) x(:), allIndices, 'UniformOutput', false);
    errInds.maxStateIndex = max(cell2mat(flatIndices));
end
