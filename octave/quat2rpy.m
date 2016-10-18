function rpy = quat2rpy(quat)
    % dcm = quat2dcm(quat);
    % rpy = [ atan2(dcm(3, 2), dcm(3, 3));
    %         asin(-dcm(3, 1));
    %         atan2(dcm(2, 1), dcm(1, 1)) ];

    q_s = quat(1, :);
    q_x = quat(2, :);
    q_y = quat(3, :);
    q_z = quat(4, :);

    rpy = [ atan2(2 * (q_y .* q_z + q_x .* q_s), 1 - 2 * (q_x.^2 + q_y.^2));
            asin(2 * (q_y .* q_s - q_x .* q_z));
            atan2(2 * (q_x .* q_y + q_z .* q_s), 1 - 2 * (q_y.^2 + q_z.^2)) ];
end
