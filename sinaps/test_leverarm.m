function test_leverarm()
    global usbl_dev_xyz; % shift in local frame of USBL relative to CRP
    global usbl_dev_dcm; % rotation of USBL in local frame
    global ahrs_dev_dcm; % rotation of AHRS in local frame

    deg2rad = pi / 180;
    wgs84();

    usbl_dev_xyz = [ 0; 0; 0 ];
    usbl_dev_dcm = rpy2dcm([ 0; 0; 0 ] * deg2rad);
    ahrs_dev_dcm = rpy2dcm([ 0; 0; 0 ] * deg2rad);

    poly = gen_poly(4, 30, 8);
    sz = size(poly, 2);
    zp = zeros(1, sz);
    op = ones(1, sz);
    wp = sin(10 * 2 * pi * (0:sz-1) / sz);

    scenter_geod = [ 50.0; 13.0; 45 ];
    tcenter_geod = [ 50.001; 13.001; 45 ];

    src_geod = poly2geod(scenter_geod, [ 150 * poly(2:-1:1, :); 5 * wp ]);
    src_rpy = [ zp; zp; wrap_2pi(poly(3, :)) ];
    tgto_geod = tcenter_geod .* op;

    %% step 1: defusion (find usbl-frame coordinates for specific loc/rot)
    usbl_dev_xyz = [ 0; 30; 0 ];
    usbl_dev_dcm = rpy2dcm([ 0; 0; 0 ] * deg2rad);
    rawm_xyz = defuse(tgto_geod, src_rpy, src_geod); % measured XYZ (got)

    % optional scale measurements
    %rawm_xyz = rawm_xyz * (1500/1465);

    % optional add noise
    %rawm_xyz = rawm_xyz + 0.5 * randn(size(rawm_xyz));

    %% step 2: fusion with another loc/rot
    usbl_dev_xyz = [ 0; 0; 0 ];
    usbl_dev_dcm = rpy2dcm([ 0; 0; 0 ] * deg2rad);
    tgt_geod = usbl_fuse(rawm_xyz, src_rpy, 0, src_geod);
    tgt_ned  = usbl_fuse(rawm_xyz, src_rpy, 0);
    plot_target(geod2wmerc(tgt_geod), geod2wmerc(src_geod), tgt_ned);

    figure(3), hold('off');
    rawc_xyz = defuse(tgto_geod, src_rpy, src_geod); % computed XYZ (should be)
    rawv_xyz = defuse(src_geod(:, 1) .* op, src_rpy, src_geod); % vessel relative coords
    plot(rawm_xyz(1, :), rawm_xyz(2, :), '.'), hold('on');
    plot(rawc_xyz(1, :), rawc_xyz(2, :), '.'), grid('on');
    title('Measurements in Device-frame');
    legend('Measured', 'Computed');

    %% step 3: try to find loc/rot based on true raw_xyz
    flip = [ 0, 1, 0; 1, 0, 0; 0, 0, -1 ];
    [ cdcm, shift ] = find_transform(flip * rawc_xyz, flip * rawm_xyz);
    disp('Shift:'); disp(shift');
    disp('Rotation:'); disp(dcm2rpy(cdcm)' / deg2rad);

    err = flip * (flip * rawc_xyz - shift - cdcm * flip * rawm_xyz);
    figure(4), plot(err(1, :), err(2, :), '.'), grid('on'), title('Calibration error');
end

function [ geod ] = poly2geod(cgeod, poly)
    geod = ecef2geod(geod2ecef(cgeod) + geod2dcm(cgeod) * poly);
end

function [ tgt_xyz ] = defuse(tgt_geod, src_rpy, src_geod)
    tgt_xyz = usbl_defuse(tgt_geod, src_rpy, 0, src_geod);
end

function plot_target(tgt, src, ned)
    k = wmerc2scale(src(:, 1));

    xt = (tgt(1, :) - src(1, 1)) / k;
    yt = (tgt(2, :) - src(2, 1)) / k;
    %zt = tgt(3, :);

    xs = (src(1, :) - src(1, 1)) / k;
    ys = (src(2, :) - src(2, 1)) / k;
    %zs = src(3, :);

    xn = -ned(2, :) + (tgt(1, 1) - src(1, 1)) / k;
    yn = -ned(1, :) + (tgt(2, 1) - src(2, 1)) / k;
    %zn = ned(3, :) + tgt(3, 1);

    t = 1:size(xt, 2);

    figure(1), hold('off');
    plot(xs, ys, '.k');
    hold('on'), grid('on');
    plot(xt, yt, '.r');
    plot(xn, yn, '.b');
    legend('CRP', 'Target', 'NED'), title('Target');

    figure(2);

    subplot(2, 1, 1), hold('off');
    plot(t, xs - xs(1), '.k');
    hold('on'), grid('on');
    plot(t, xt - xt(1), '.r');
    plot(t, xn - xn(1), '.b');
    legend('CRP', 'Target', 'NED'), title('North');

    subplot(2, 1, 2), hold('off');
    plot(t, ys - ys(1), '.k');
    hold('on'), grid('on');
    plot(t, yt - yt(1), '.r');
    plot(t, yn - yn(1), '.b');
    legend('CRP', 'Target', 'NED'), title('East');
end
