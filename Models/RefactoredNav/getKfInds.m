function inds = getKfInds()

    % Quaternion
    inds.qw = 1;
    inds.qx = 2;
    inds.qy = 3;
    inds.qz = 4;
    inds.quat = [inds.qw; inds.qx; inds.qy; inds.qz];

    % ECEF Position
    inds.px = 5;
    inds.py = 6;
    inds.pz = 7;
    inds.pos = [inds.px; inds.py; inds.pz]; % [m]
  
    % ECEF Velocity
    inds.vx = 8; 
    inds.vy = 9;
    inds.vz = 10;
    inds.vel = [inds.vx; inds.vy; inds.vz]; % [m/s]

    % Gyro Bias
    inds.gbx = 11;
    inds.gby = 12;
    inds.gbz = 13;
    inds.gyroBias = [inds.gbx; inds.gby; inds.gbz]; % [rad/s]

    % Accel Bias
    inds.abx = 14;
    inds.aby = 15;
    inds.abz = 16;
    inds.accelBias = [inds.abx; inds.aby; inds.abz]; % [m/s/s]

    % Mag Bias
    inds.mbx = 17;
    inds.mby = 18;
    inds.mbz = 19;
    inds.magBias = [inds.mbx; inds.mby; inds.mbz]; % [uT]

    % Gravity 
    inds.g = 20;

    % Compute max scalar index
    scalarFields = structfun(@(x) isscalar(x), inds);
    scalarValues = struct2cell(inds);
    inds.maxStateIndex = max(cell2mat(scalarValues(scalarFields)));
end