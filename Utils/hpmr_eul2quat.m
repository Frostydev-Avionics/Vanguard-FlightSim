function q = hpmr_eul2quat(roll, pitch, yaw)
% Converts ZYX Euler angles (roll, pitch, yaw) to quaternion [w x y z]

cr = cos(roll/2);
sr = sin(roll/2);
cp = cos(pitch/2);
sp = sin(pitch/2);
cy = cos(yaw/2);
sy = sin(yaw/2);

q = [
    cr*cp*cy + sr*sp*sy;  % qw
    sr*cp*cy - cr*sp*sy;  % qx
    cr*sp*cy + sr*cp*sy;  % qy
    cr*cp*sy - sr*sp*cy;  % qz
];
end
