function test_calib()
    global usbl_dev_xyz; % shift in local frame of USBL relative to GNSS
    global usbl_dev_dcm; % rotation of USBL in local frame
    global ahrs_dev_dcm; % rotation of AHRS in local frame

    d2r = pi / 180;
    wgs84();

    usbl_dev_xyz = [ 0.5; 0; 2 ];
    usbl_dev_dcm = rpy2dcm([ 0; 0; 75 ] * d2r);
    ahrs_dev_dcm = rpy2dcm([ 0; 0; 0 ] * d2r);

    %% test usbl calibration (generate vessel locations)
    n = 100;
    cgeod = [ 50; 13; 45 ]; % center [ lat, lon, alt ]
    cecef = geod2ecef(cgeod);
    edcm = geod2dcm(cgeod);

    %% generate crp positions and rotations
    [ pts, srpy ] = track(n, 0.5, 100, [ -2; 4; 0 ] * d2r);
    secef = edcm * pts + cecef;
    sgeod = ecef2geod(secef);
    sdcm = rpy2dcm(srpy);

    %% generate target locations (25m west, 20m deep)
    noise = 0.5 * (rand(3, n) - 0.5);
    tecef = edcm * [ -25; 0; -20 ] + cecef + noise;
    tgeod = ecef2geod(tecef);

    %% compute usbl-frame target coordinates
    noise = 0.25 * (rand(3, n) - 0.5);
    txyz = usbl_defuse(tgeod, srpy, 0, sgeod) + noise;

    %% find rotation and shift
    shift = [ 0; 0; 2 ]; % initial values
    [ cdcm, cshift, ntri ] = usbl_calib(shift_src(secef, edcm, sdcm, shift), sdcm, txyz, 25);

    %% display results
    if ntri == 0
        disp('No solutions found');
    else
        shift = shift + cshift;

        disp('Triangles:'); disp(ntri);
        disp('Shift:'); disp(shift');
        disp('Rotation:'); disp(dcm2rpy(cdcm)' / d2r);
    end
end

function [ pts, rpy ] = track(n, heave, radius, irpy)
    i = (1:n).^2;
    phi = 0.75 * 2 * pi .* (i - 1) ./ n^2;
    theta = 5 * pi .* (1 - i) ./ n^2;
    pts = [ radius * cos(phi); radius * sin(phi); 0.5 * heave * sin(theta) ];
    rpy = [ zeros(1, n); -0.03 * sin(theta); wrap_2pi(-phi - pi/2) ] + irpy;
end

function msrc = shift_src(src, edcm, dcm, shift)
    ned2enu = [ 0, 1, 0; 1, 0, 0; 0, 0, -1 ];

    msrc = zeros(size(src));
    for i = 1:size(src, 2)
        msrc(:, i) = src(:, i) + edcm * ned2enu * dcm(:, :, i) * shift;
    end
end
