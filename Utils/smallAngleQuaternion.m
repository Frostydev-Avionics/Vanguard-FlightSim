function dq = smallAngleQuaternion(dtheta)
    angle = norm(dtheta);
    if angle < 1e-8
        dq = [1; 0; 0; 0];
    else
        axis = dtheta / angle;
        dq = [cos(angle/2); axis * sin(angle/2)];
    end
end