function dcm = quat2dcm(quat)
    q_s = quat(1);
    q_x = quat(2);
    q_y = quat(3);
    q_z = quat(4);

    dcm = zeros(3, 3);

    dcm(1, 1) = 1 - 2 * (q_y^2 + q_z^2);
    dcm(2, 2) = 1 - 2 * (q_x^2 + q_z^2);
    dcm(3, 3) = 1 - 2 * (q_x^2 + q_y^2);

    dcm(1, 2) = 2 * (q_x * q_y - q_z * q_s);
    dcm(1, 3) = 2 * (q_x * q_z + q_y * q_s);
    dcm(2, 1) = 2 * (q_x * q_y + q_z * q_s);
    dcm(2, 3) = 2 * (q_y * q_z - q_x * q_s);
    dcm(3, 1) = 2 * (q_x * q_z - q_y * q_s);
    dcm(3, 2) = 2 * (q_y * q_z + q_x * q_s);
end
