function consts = getKfConsts()

    asm330.accelStdDev = 2.1582e-3; % [m/s/s]
    asm330.gyroStdDev  = 4.4e-6; % [rad/s]
    asm330.gyroBiasStdDev = 4.9e-5; % [rad/s] Bias Instability Guess
    asm330.quatStdDev = deg2rad(1); % [rad] Quaternion Error
    asm330.accelBiasStdDev = 1e-3;

    consts.asm330 = asm330;

    consts.navPropDt = 0.01; % [s] Nav Prop Rate

    icm20948.accelXY_var = 0.0383; % [g]
    icm20948.accelZ_var  = 0.0626; % [g]
    icm20948.accelXY_VRW = 0.0052; % [g/sqrt(hz)]
    icm20948.accelZ_VRW  = 0.0099; % [g/sqrt(hz)]
    icm20948.gyroXYZ_var = 0.0051; % [deg/s]
    icm20948.gyro_VRW    = 8.33e-4; % [deg/s/sqrt(Hz)]
    icm20948.magXYZ_var  = 0.7263;  % [uT]

    mmc5983.magXYZ_var = 0.4; % [uT]

    consts.R_grav = 0.5 * eye(3); % [m/s/s]

    consts.R_gps   = 2 * eye(3); % [m]

    consts.icm20948 = icm20948;

    consts.mmc5983 = mmc5983;

end