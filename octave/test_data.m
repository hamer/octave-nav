function test_data(name, addr)
    deg2rad = pi / 180;
    wgs84();

    data = csvread(name, 0, 1);
    data = data(find(data(:, 37) ~= 0), :); % filter out records without gps
    data = data(find(data(:, 3) == addr), :); % filter out records without gps
    nlines = size(data, 1);

    raw_xyz = data(:, 10:12)';      % XYZ coordinates of a target in USBL frame
    src_rpy = data(:, 29:31)';      % vessels roll/pitch/yaw
    crp_geod = data(:, 37:39)';     % lat, lon and alt of CRP (not a GPS antenna)

    %% prepare initial data
    xyz = [ 0, 1, 0; 1, 0, 0; 0, 0, -1 ] * raw_xyz;
    sdcm = rpy2dcm(src_rpy);
    src = geod2ecef(crp_geod);

    %% find rotation and shift
    dcm = rpy2dcm([ 0; 0; 0 ] * deg2rad); % initial values
    shift = [ 0; 0; 2 ];

    disp(''); disp([ '=========== Run C ===========' ]);
    [ cdcm, cshift ] = usbl_calib(shift_src(src, sdcm, shift), sdcm, dcm * xyz);

    if isempty(cdcm)
        disp('No solution found');
        return;
    end

    disp('Rotation:'); disp(dcm2rpy(cdcm)' / deg2rad);
    disp('Shift:'); disp(cshift');
end

function msrc = shift_src(src, dcm, shift)
    ned2enu = [ 0, 1, 0; 1, 0, 0; 0, 0, -1 ];

    msrc = [];
    for i = 1:size(src, 2)
        ecef_dcm = geod2dcm(ecef2geod(src(:, i)));
        src_dcm = dcm(:, :, i);

        msrc = [ msrc, src(:, i) + ecef_dcm * ned2enu * src_dcm * shift ];
    end
end
