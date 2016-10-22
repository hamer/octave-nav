function [ ntri, dcm, shift ] = test_data(name, addr, ishift, talt)
    wgs84();

    data = csvread(name, 0, 1);
    data = data(find(data(:, 37) ~= 0), :);     % filter out records without gps
    data = data(find(data(:, 3) == addr), :);   % filter out records with wrong address
    nlines = size(data, 1);

    raw_xyz = data(:, 10:12)';      % XYZ coordinates of a target in USBL frame
    src_rpy = data(:, 29:31)';      % vessels Roll/Pitch/Yaw
    crp_geod = data(:, 37:39)';     % Lat, Lon and Alt of CRP

    %% prepare initial data
    xyz = [ 0, 1, 0; 1, 0, 0; 0, 0, -1 ] * raw_xyz;
    sdcm = rpy2dcm(src_rpy);
    src = geod2ecef(crp_geod);

    %% find rotation and shift
    dcm = [ 1, 0, 0; 0, 1, 0; 0, 0, 1 ];

    if nargin > 2
        shift = ishift';
    else
        shift = [ 0; 0; 0 ];
    end

    if nargin > 3
        [ cdcm, cshift, ntri ] = usbl_calib(shift_src(src, sdcm, shift), sdcm, dcm * xyz, talt);
    else
        [ cdcm, cshift, ntri ] = usbl_calib(shift_src(src, sdcm, shift), sdcm, dcm * xyz);
    end

    if ntri == 0
        disp('No solutions found');
    else
        dcm = cdcm * dcm;
        shift = shift + cshift;

        disp('Triangles:'); disp(ntri);
        disp('Rotation:'); disp(dcm2rpy(dcm)' / deg2rad);
        disp('Shift:'); disp(shift');
    end
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
