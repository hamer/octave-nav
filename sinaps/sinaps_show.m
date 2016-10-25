function sinaps_show(name, addr, rot, shift)
    global usbl_dev_xyz; % shift in local frame of USBL relative to CRP
    global usbl_dev_dcm; % rotation of USBL in local frame
    global ahrs_dev_dcm; % rotation of AHRS in local frame

    deg2rad = pi / 180;
    wgs84();

    usbl_dev_xyz = shift';
    usbl_dev_dcm = rpy2dcm(rot' * deg2rad);
    ahrs_dev_dcm = rpy2dcm([ 0; 0; 0 ] * deg2rad);

    data = csvread(name, 0, 1);
    data = data(find(data(:, 37) ~= 0), :);     % filter out records without gps
    data = data(find(data(:, 3) == addr), :);   % filter out records with wrong address
    nlines = size(data, 1);

    raw_xyz = data(:, 10:12)';      % XYZ coordinates of a target in USBL frame
    src_rpy = data(:, 29:31)';      % vessels Roll/Pitch/Yaw
    crp_geod = data(:, 37:39)';     % Lat, Lon and Alt of CRP

    xyz = [ 0, 1, 0; 1, 0, 0; 0, 0, -1 ] * raw_xyz;

    n = size(xyz, 2);
    tgt_ecef = zeros(3, n);

    for i = 1:n
        tgt_ecef(:, i) = usbl_fuse(xyz(:, i), src_rpy(:, i), crp_geod(:, i));
    end

    plot_target(geod2wmerc(ecef2geod(tgt_ecef)), geod2wmerc(crp_geod));
end

function plot_target(tgt, src)
    k = wmerc2scale(src(:, 1));

    xt = (tgt(1, :) - src(1, 1)) / k;
    yt = (tgt(2, :) - src(2, 1)) / k;

    xs = (src(1, :) - src(1, 1)) / k;
    ys = (src(2, :) - src(2, 1)) / k;

    figure(1), hold('off');
    plot(xs, ys, '.k');
    hold('on'), grid('on');
    plot(xt, yt, '.r');
    legend('CRP', 'Target'), title('Target');
end
