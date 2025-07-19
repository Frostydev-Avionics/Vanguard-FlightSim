function sf = accel2sf(a_b, R_NB)
% ACCEL2SF - Computes specific force from linear accel and gravity.
% INPUTS:
%   a_b - [3x1] Linear acceleration in body frame
%   R_NB - [3x3] Rotation matrix from Body to NED
% OUTPUTS:
%   sf - [3x1] Specific force in body frame (excludes gravity)

    g_NED = [0; 0; 9.80665];  % Gravity in NED
    g_B = R_NB * g_NED;       % Rotate gravity to body frame

    sf = a_b - g_B;
end
