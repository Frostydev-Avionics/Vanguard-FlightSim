function q_0 = computeInitialQuaternion(a_b, m_b)

    % === Normalize accelerometer ===
    a_b = a_b / norm(a_b);
    ax = a_b(1);
    ay = a_b(2);
    az = a_b(3);

    % === Estimate Roll and Pitch ===
    roll  = atan2(ay, az);
    pitch = atan2(ax, sqrt(ay^2 + az^2));

    % === Rotate mag vector into horizontal plane (tilt compensation) ===
    m_b = m_b / norm(m_b + 1e-8);  % avoid divide-by-zero
    mx = m_b(1);
    my = m_b(2);
    mz = m_b(3);

    % Tilt compensation
    mag_x =  mx * cos(pitch) + mz * sin(pitch);
    mag_y =  mx * sin(roll)*sin(pitch) + ...
             my * cos(roll) - ...
             mz * sin(roll)*cos(pitch);

    yaw = atan2(-mag_y, mag_x);  % -my, mx for NED

    % === Convert to quaternion (ZYX: yaw → pitch → roll) ===
    cy = cos(yaw / 2);   sy = sin(yaw / 2);
    cp = cos(pitch / 2); sp = sin(pitch / 2);
    cr = cos(roll / 2);  sr = sin(roll / 2);

    qw = cr * cp * cy + sr * sp * sy;
    qx = sr * cp * cy - cr * sp * sy;
    qy = cr * sp * cy + sr * cp * sy;
    qz = cr * cp * sy - sr * sp * cy;

    q_0 = [qw; qx; qy; qz];  % [w; x; y; z]
    q_0 = q_0 / norm(q_0);

    % Attitude Initialization
    yaw_0 = deg2rad(0);
    roll_0 = deg2rad(0);
    pitch_0 = deg2rad(86);
     
    eul_0 = [roll_0; pitch_0; yaw_0];
    
    q_0 = hpmr_eul2quat(roll_0, pitch_0, yaw_0);
end
