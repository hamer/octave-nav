function test_fuse()
    global usbl_dev_xyz; % shift in local frame of USBL relative to CRP
    global usbl_dev_dcm; % rotation of USBL in local frame
    global ahrs_dev_dcm; % rotation of AHRS in local frame

    deg2rad = pi / 180;
    wgs84();

    usbl_dev_xyz = [ 0; 0; 0 ];
    usbl_dev_dcm = rpy2dcm([ 0; 0; 0 ] * deg2rad);
    ahrs_dev_dcm = rpy2dcm([ 0; 0; 0 ] * deg2rad);

    %% test different convertion modes
    % usbl_fuse([ 0; 0; 0 ], [ 0; 0; 0 ] * deg2rad, 0, [ 0; 0; 0 ])
    % usbl_fuse([ 0; 0; 0 ], [ 0; 0; 0 ] * deg2rad, 0)
    % usbl_fuse([ 0; 0; 0 ])

    %% test forward/backward fusion
    tgt_usbl_xyz = [ 0; 0; 0 ];
    src_ahrs_rpy = [ 0; 0; 0 ] * deg2rad;
    src_geod = [ 0; 0; 0 ];
    [ tgt_geod, src_dcm ] = usbl_fuse(tgt_usbl_xyz, src_ahrs_rpy, 0, src_geod);
    tgt_back_xyz = usbl_defuse(tgt_geod, src_ahrs_rpy, 0, src_geod);
end
