function test_fuse()
    global usbl_dev_xyz; % shift in local frame of USBL relative to GNSS
    global usbl_dev_dcm; % rotation of USBL in local frame
    global ahrs_dev_dcm; % rotation of AHRS in local frame

    deg2rad = pi / 180;
    wgs84();

    usbl_dev_xyz = [ 0; 0; 0 ];
    usbl_dev_dcm = rpy2dcm([ 0; 0; 30 ] * deg2rad);
    ahrs_dev_dcm = rpy2dcm([ 0; 0; 0 ] * deg2rad);

    %% test different convertion modes
    % usbl_fuse([ 0; 0; 0 ], [ 0; 0; 0 ] * deg2rad, [ 0; 0; 0 ])
    % usbl_fuse([ 0; 0; 0 ], [ 0; 0; 0 ] * deg2rad)
    % usbl_fuse([ 0; 0; 0 ])

    %% test forward/backward fusion
    % tgt_usbl_xyz = [ 0; 0; 0 ];
    % src_ahrs_rpy = [ 0; 0; 0 ] * deg2rad;
    % src_geod = [ 0; 0; 0 ];
    % [ tgt_ecef, src_dcm, src_ecef ] = usbl_fuse(tgt_usbl_xyz, src_ahrs_rpy, src_geod);
    % tgt_back_xyz = usbl_defuse(tgt_ecef, src_ahrs_rpy, src_geod);

    %% test usbl calibration (generate vessel locations)
    cnt_geod = [ 50; 13; 49 ]; % center [ lat, lon, alt ]
    [ src_geod, src_rpy ] = around(cnt_geod, 100, 100);
    src_rpy = src_rpy + [ 5; -5; 0 ] * deg2rad;

    %% generate target locations (25m west, 20m deep)
    tgt_wmerc = geod2wmerc(cnt_geod) + [ -25 * geod2scale(cnt_geod); 0; -20 ];
    tgt_ecef = zeros(3, size(src_geod, 2)) + geod2ecef(wmerc2geod(tgt_wmerc));
    tgt_ecef = tgt_ecef + 0.25 * (rand(size(tgt_ecef)) - 0.5);

    %% compute required data (ECEF for vessel, LF for target and vessel rotations)
    src = geod2ecef(src_geod);
    quat = dcm2quat(rpy2dcm(src_rpy));
    xyz = [];
    for i = 1:size(tgt_ecef, 2)
        xyz = [ xyz, usbl_defuse(tgt_ecef(:, i), src_rpy(:, i), src_geod(:, i)) + 0.25 * (rand(3, 1) - 0.5) ];
    end

    % figure(1), plot(xyz(1, :), xyz(2, :), 'r.');
    % xlim([ -50, 50 ]), ylim([ -150, -50 ]), grid('on');

    %% find rotation and shift
    dcm = rpy2dcm([ 0; 0; 0 ] * deg2rad); % initial values
    shift = [ 0; 0; 0 ];

    disp(''); disp('=========== Run M ===========');
    [ dcm1, shift1 ] = usbl_calib(shift_src(src, quat, shift), quat, dcm * xyz);
    disp('Rotation:'); disp(dcm2rpy(dcm1)' / deg2rad);
    disp('Shift:'); disp(shift1');

    yawdcm = rpy2dcm([ 0; 0; dcm2rpy(dcm1)(3) ]); % ignore roll/pitch
    shift = yawdcm * shift + shift1;
    dcm = yawdcm * dcm;

    disp(''); disp('Press a key to continue...'), pause();

    disp(''); disp('=========== Run M ===========');
    [ dcm2, shift2 ] = usbl_calib(shift_src(src, quat, shift), quat, dcm * xyz);
    disp('Rotation:'); disp(dcm2rpy(dcm2)' / deg2rad);
    disp('Shift:'); disp(shift2');

    yawdcm = rpy2dcm([ 0; 0; dcm2rpy(dcm2)(3) ]); % ignore roll/pitch
    shift = yawdcm * shift + shift2;
    dcm = yawdcm * dcm;

    disp(''); disp('Press a key to continue...'), pause();

    disp(''); disp('=========== Run F ===========');
    [ dcm3, shift3 ] = usbl_calib(shift_src(src, quat, shift), quat, dcm * xyz);
    disp('Rotation:'); disp(dcm2rpy(dcm3)' / deg2rad);
    disp('Shift:'); disp(shift3');

    shift = dcm3 * shift + shift3;
    dcm = dcm3 * dcm;

    disp(''); disp('Press a key to continue...'), pause();

    disp(''); disp('=========== Run N ===========');
    [ dcm4, shift4 ] = usbl_calib(shift_src(src, quat, shift), quat, dcm * xyz);
    disp('Rotation:'); disp(dcm2rpy(dcm4)' / deg2rad);
    disp('Shift:'); disp(shift4');

    shift = dcm4 * shift + shift4;
    dcm = dcm4 * dcm;

    disp(''); disp('Press a key to continue...'), pause();

    disp(''); disp('=========== Total ===========');
    disp('Rotation:'); disp(dcm2rpy(dcm)' / deg2rad);
    disp('Shift:'); disp(shift');
end

function [ pts, rpy ] = circ(n)
    i = 1:n;
    phi = 0.75 * 2 * pi .* (i - 1) ./ n;
    pts = [ cos(phi); sin(phi); zeros(1, n) ];
    rpy = [ zeros(2, n); wrap_2pi(-phi - pi/2) ];
end

function [ s_geod, s_rpy ] = around(c_geod, r, n)
    k = geod2scale(c_geod);
    c_wmerc = geod2wmerc(c_geod);
    [ pts, s_rpy ] = circ(n);
    s_wmerc = pts .* [ r * k; r * k; 1 ] + c_wmerc;
    s_geod = wmerc2geod(s_wmerc);
end

function msrc = shift_src(src, quat, shift)
    ned2enu = [ 0, 1, 0; 1, 0, 0; 0, 0, -1 ];

    msrc = [];
    for i = 1:size(src, 2)
        ecef_dcm = geod2dcm(ecef2geod(src(:, i)));
        src_dcm = quat2dcm(quat(:, i));

        msrc = [ msrc, src(:, i) + ecef_dcm * ned2enu * src_dcm * shift ];
    end
end
