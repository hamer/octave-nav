function imc_guide(emu, offset)
    global usbl_dev_xyz; % shift in local frame of USBL relative to CRP
    global usbl_dev_dcm; % rotation of USBL in local frame
    global ahrs_dev_dcm; % rotation of AHRS in local frame

    wgs84();
    usbl_dev_xyz = zeros(3, 1);
    usbl_dev_dcm = eye(3);
    ahrs_dev_dcm = eye(3);

    geod = usbl_fuse(offset', [ 0; 0; 0 ], 0, [ 50; 13.5; 0 ]);
    emu.guide(geod(1), geod(2), geod(3), 1.2);
end
