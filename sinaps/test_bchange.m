function [ irpy_dcm, irpy_form ] = test_bchange(rpy)
    flip = [ 0, 1, 0; 1, 0, 0; 0, 0, -1 ];
    irpy_dcm = dcm2rpy(flip' * rpy2dcm(rpy) * flip);

    cr = cos(rpy(1)); cp = cos(rpy(2)); cy = cos(rpy(3));
    sr = sin(rpy(1)); sp = sin(rpy(2)); sy = sin(rpy(3));

    irpy_form = [ atan2(sp, cr * cp); ...
                  asin(sr * cp); ...
                  atan2(sr * sp * cy - cr * sy, sr * sp * sy + cr * cy) ];

    r2d = 180/pi;
    disp('DCM-trans:'); disp(irpy_dcm' * r2d);
    disp('Formal trans:'); disp(irpy_form' * r2d);
end
