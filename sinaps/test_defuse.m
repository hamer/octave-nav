function test_defuse()
    global usbl_dev_xyz; % shift in local frame of USBL relative to CRP
    global usbl_dev_dcm; % rotation of USBL in local frame
    global ahrs_dev_dcm; % rotation of AHRS in local frame

    deg2rad = pi / 180;
    wgs84();

    usbl_dev_xyz = [ 0; 0; 0 ];
    usbl_dev_dcm = rpy2dcm([ 0; 0; 0 ] * deg2rad);
    ahrs_dev_dcm = rpy2dcm([ 0; 0; 0 ] * deg2rad);

    poly = gen_poly(4, 50, 50);
    sz = size(poly, 2);
    zp = zeros(1, sz);
    op = ones(1, sz);
    wp = sin(10 * 2 * pi * (0:sz-1) / sz);

    center_geod = [ 50.0; 13.0; 45 ];

    src_geod = poly2geod(center_geod, [ 150 * poly(2:-1:1, :); 5 * wp ]);
    src_rpy = [ zp; zp; wrap_2pi(poly(3, :)) ];
    tgt_geod = center_geod .* op;

    hold('off'), plot_tgt(defuse(tgt_geod, src_rpy, src_geod)), grid('on');

    usbl_dev_xyz = [ 30; 30; 0 ];
    hold('on'), plot_tgt(defuse(tgt_geod, src_rpy, src_geod));
end

function [ geod ] = poly2geod(cgeod, poly)
    geod = ecef2geod(geod2ecef(cgeod) + geod2dcm(cgeod) * poly);
end

function plot_tgt(tgt_xyz)
    plot3(tgt_xyz(1, :), tgt_xyz(2, :), tgt_xyz(3, :), '.');
end

function [ tgt_xyz ] = defuse(tgt_geod, src_rpy, src_geod)
    tgt_xyz = usbl_defuse(tgt_geod, src_rpy, 0, src_geod);
end
