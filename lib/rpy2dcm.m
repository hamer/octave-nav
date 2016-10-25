function dcm = rpy2dcm(rpy)
    cr = cos(rpy(1, :)); cp = cos(rpy(2, :)); cy = cos(rpy(3, :));
    sr = sin(rpy(1, :)); sp = sin(rpy(2, :)); sy = sin(rpy(3, :));

    % rx = [  1,   0,   0;
    %         0,  cr, -sr;
    %         0,  sr,  cr ];

    % ry = [ cp,   0,  sp;
    %         0,   1,   0;
    %       -sp,   0,  cp ];

    % rz = [ cy, -sy,   0;
    %        sy,  cy,   0;
    %         0,   0,   1 ];

    % dcm = rz * ry * rx;
    % dcm = [ cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy;
    %         cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy;
    %             -sp,                sr * cp,                cr * cp ];

    dcms = [ cp .* cy; sr .* sp .* cy - cr .* sy; cr .* sp .* cy + sr .* sy;
             cp .* sy; sr .* sp .* sy + cr .* cy; cr .* sp .* sy - sr .* cy;
                  -sp;                  sr .* cp;                  cr .* cp ];
    dcm = permute(reshape(dcms, 3, 3, size(rpy, 2)), [ 2, 1, 3 ]);
end
