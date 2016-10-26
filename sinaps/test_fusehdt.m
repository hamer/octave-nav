function test_fusehdt()
    d2r = pi / 180;
    flip = [ 0, 1, 0; 1, 0, 0; 0, 0, -1];

    n = 10;
    i = 1:n;

    % prepare the same rotations for both coordinate-systems
    hdt = 20;
    dcm_zu2enu_c = rpy2dcm([ 30; 15; hdt ] * d2r);
    dcm_zd2ned_c = flip * dcm_zu2enu_c * flip;

    % get roll/pitch from z-up frame ang heading from z-down frame
    rpy = dcm2rpy(dcm_zu2enu_c);
    dcm_rp = flip * rpy2dcm([ rpy(1); rpy(2); 0 ]) * flip; % note the phantom yaw
    dcm_y = rpy2dcm([ 0; 0; -hdt ] * d2r);
    dcm_zd2ned_i = dcm_y * dcm_rp;

    % make initial set of points
    zu_point = zeros(3, 3*n);
    zu_point(1, 1:n) = i;
    zu_point(2, (n/2:n) + n) = i(n/2:n);
    zu_point(3, (1:n/2) + 2*n) = i(1:n/2);
    zd_point = flip * zu_point;

    % convert to enu and ned
    zu_enu_c = dcm_zu2enu_c * zu_point;
    zd_ned_c = dcm_zd2ned_c * zd_point;
    zd_ned_i = dcm_zd2ned_i * zd_point;

    disp('Point in Z-up:'), disp(zu_point(:, 10)');
    disp('Point in Z-down:'), disp(zd_point(:, 10)');
    disp('');
    disp('Point from Z-up in ENU:'), disp(zu_enu_c(:, 10)');
    disp('Point from Z-down in NED:'), disp(zd_ned_c(:, 10)');
    disp('Point from Z-down in NED(fused):'), disp(zd_ned_i(:, 10)');
    disp('');
    disp('Euler Z-up -> ENU:'), disp(dcm2rpy(dcm_zu2enu_c)' / d2r);
    disp('Euler Z-down -> NED:'), disp(dcm2rpy(dcm_zd2ned_c)' / d2r);
    disp('Euler Z-down -> NED(fused):'), disp(dcm2rpy(dcm_zd2ned_i)' / d2r);
    disp('');
    disp('Quaternion Z-up -> ENU:'), disp(dcm2quat(dcm_zu2enu_c)');
    disp('Quaternion Z-down -> NED:'), disp(dcm2quat(dcm_zd2ned_c)');
    disp('Quaternion Z-down -> NED(fused):'), disp(dcm2quat(dcm_zd2ned_i)');

    % figure(1), hold('off');
    % plot3(zd_ned_c(1, :), zd_ned_c(2, :), zd_ned_c(3, :), '.r');
    % hold('on'), grid('on');
    % plot3(zd_ned_i(1, :), zd_ned_i(2, :), zd_ned_i(3, :), '.k');
end
