function quat = rpy2quat(rpy)
    % q_x = axis2quat(rpy(1), [ 1; 0; 0 ]);
    % q_y = axis2quat(rpy(2), [ 0; 1; 0 ]);
    % q_z = axis2quat(rpy(3), [ 0; 0; 1 ]);
    % quat = quat_mult(q_z, quat_mult(q_y, q_x));

    cr = cos(rpy(1, :) / 2); cp = cos(rpy(2, :) / 2); cy = cos(rpy(3, :) / 2);
    sr = sin(rpy(1, :) / 2); sp = sin(rpy(2, :) / 2); sy = sin(rpy(3, :) / 2);

    quat = [ cr .* cp .* cy + sr .* sp .* sy;
             sr .* cp .* cy - cr .* sp .* sy;
             cr .* sp .* cy + sr .* cp .* sy;
             cr .* cp .* sy - sr .* sp .* cy ];
end
