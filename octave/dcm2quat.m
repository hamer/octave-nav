function quat = dcm2quat(dcm)
    [ m, i ] = max([ 1 + dcm(1, 1) + dcm(2, 2) + dcm(3, 3), ...
                     1 + dcm(1, 1) - dcm(2, 2) - dcm(3, 3), ...
                     1 - dcm(1, 1) + dcm(2, 2) - dcm(3, 3), ...
                     1 - dcm(1, 1) - dcm(2, 2) + dcm(3, 3) ]);

    switch i
    case 1
        q_s = sqrt(0.25 * m);
        q_x = 0.25 * (dcm(3, 2) - dcm(2, 3)) / q_s;
        q_y = 0.25 * (dcm(1, 3) - dcm(3, 1)) / q_s;
        q_z = 0.25 * (dcm(2, 1) - dcm(1, 2)) / q_s;

    case 2
        q_x = sqrt(0.25 * m);
        q_s = 0.25 * (dcm(3, 2) - dcm(2, 3)) / q_x;
        q_y = 0.25 * (dcm(2, 1) + dcm(1, 2)) / q_x;
        q_z = 0.25 * (dcm(1, 3) + dcm(3, 1)) / q_x;

    case 3
        q_y = sqrt(0.25 * m);
        q_s = 0.25 * (dcm(1, 3) - dcm(3, 1)) / q_y;
        q_x = 0.25 * (dcm(2, 1) + dcm(1, 2)) / q_y;
        q_z = 0.25 * (dcm(3, 2) + dcm(2, 3)) / q_y;

    case 4
        q_z = sqrt(0.25 * m);
        q_s = 0.25 * (dcm(2, 1) - dcm(1, 2)) / q_z;
        q_x = 0.25 * (dcm(1, 3) + dcm(3, 1)) / q_z;
        q_y = 0.25 * (dcm(3, 2) + dcm(2, 3)) / q_z;
    end

    quat = [ q_s; q_x; q_y; q_z ];
end
