function quat = dcm2quat(dcm)
    n = size(dcm, 3);
    quat = zeros(4, size(dcm, 3));

    for i = 1:n
        [ m, v ] = max([ 1 + dcm(1, 1, i) + dcm(2, 2, i) + dcm(3, 3, i), ...
                         1 + dcm(1, 1, i) - dcm(2, 2, i) - dcm(3, 3, i), ...
                         1 - dcm(1, 1, i) + dcm(2, 2, i) - dcm(3, 3, i), ...
                         1 - dcm(1, 1, i) - dcm(2, 2, i) + dcm(3, 3, i) ]);

        switch v
        case 1
            q_s = sqrt(0.25 * m);
            q_x = 0.25 * (dcm(3, 2, i) - dcm(2, 3, i)) / q_s;
            q_y = 0.25 * (dcm(1, 3, i) - dcm(3, 1, i)) / q_s;
            q_z = 0.25 * (dcm(2, 1, i) - dcm(1, 2, i)) / q_s;

        case 2
            q_x = sqrt(0.25 * m);
            q_s = 0.25 * (dcm(3, 2, i) - dcm(2, 3, i)) / q_x;
            q_y = 0.25 * (dcm(2, 1, i) + dcm(1, 2, i)) / q_x;
            q_z = 0.25 * (dcm(1, 3, i) + dcm(3, 1, i)) / q_x;

        case 3
            q_y = sqrt(0.25 * m);
            q_s = 0.25 * (dcm(1, 3, i) - dcm(3, 1, i)) / q_y;
            q_x = 0.25 * (dcm(2, 1, i) + dcm(1, 2, i)) / q_y;
            q_z = 0.25 * (dcm(3, 2, i) + dcm(2, 3, i)) / q_y;

        case 4
            q_z = sqrt(0.25 * m);
            q_s = 0.25 * (dcm(2, 1, i) - dcm(1, 2, i)) / q_z;
            q_x = 0.25 * (dcm(1, 3, i) + dcm(3, 1, i)) / q_z;
            q_y = 0.25 * (dcm(3, 2, i) + dcm(2, 3, i)) / q_z;
        end
        quat(:, i) = [ q_s; q_x; q_y; q_z ];
    end
end
