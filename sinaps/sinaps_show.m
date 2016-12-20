%
% SiNAPS logs visualizes
%
% Arguments:
%   * Filename (e.g. '2016-10-21-data-01.csv'). Only USBL lines should be exported;
%   * Acoustic address for calibrating target;
%   * USBL-rotation (Roll/Pitch/Yaw)
%   * USBL-location (X, Y, Z)
%   * AHRS-rotation (Roll/Pitch/Yaw) or 'fused'
%   * External heading flag (use fused heading as it was from GNSS)
%
% Example:
%   sinaps_show('test-data.csv', 2, [ 0, 0, -132 ], [ 0, 0, 2 ], [ 0, 0, -132 ], 1);
%   sinaps_show('test-data.csv', 2, [ 0, 0, -132 ], [ 0, 0, 2 ], 'fused');
%
function sinaps_show(name, addr, urot, ushift, arot, is_enu, ehdt_flag)
    global usbl_dev_xyz; % shift in local frame of USBL relative to CRP
    global usbl_dev_dcm; % rotation of USBL in local frame
    global ahrs_dev_dcm; % rotation of AHRS in local frame

    deg2rad = pi / 180;
    wgs84();

    if nargin < 5 || strcmp(arot, 'fused') == 1
        ahrs_dev_dcm = [ 1, 0, 0; 0, 1, 0; 0, 0, 1 ];
    else
        ahrs_dev_dcm = rpy2dcm(arot' * deg2rad);
    end

    if nargin < 6
        is_enu = 0;
    end

    usbl_dev_xyz = ushift';
    usbl_dev_dcm = rpy2dcm(urot' * deg2rad);

    data = csvread(name, 0, 1);
    data = data(find(data(:, 37) ~= 0), :);     % filter out records without gps
    data = data(find(data(:, 3) == addr), :);   % filter out records with wrong address
    nlines = size(data, 1);

    raw_xyz = data(:, 10:12)';      % XYZ coordinates of a target in USBL frame
    crp_geod = data(:, 37:39)';     % Lat, Lon and Alt of CRP

    if strcmp(arot, 'fused') == 1
        src_rpy = data(:, 29:31)';  % vessels Roll/Pitch/Yaw
    elseif nargin < 7 || ehdt_flag ~= 1
        src_rpy = data(:, 26:28)';  % raw Roll/Pitch/Yaw
    else
        src_rpy = data(:, [ 26:28, 31 ])';  % raw Roll/Pitch/Yaw + vessel Yaw
    end

    tgt_geod = usbl_fuse(raw_xyz, src_rpy, is_enu, crp_geod);
    [ tgt_ned, dcms ] = usbl_fuse(raw_xyz, src_rpy, is_enu);
    plot_target(geod2wmerc(tgt_geod), geod2wmerc(crp_geod), tgt_ned);

    rpys = dcm2rpy(dcms);
    figure(3);
    subplot(2, 2, 1), plot((src_rpy(1, :)) / deg2rad, '-b'), grid, title('Raw roll');
    subplot(2, 2, 3), plot((src_rpy(2, :)) / deg2rad, '-b'), grid, title('Raw pitch');
    subplot(2, 2, 2), plot((rpys(1, :)) / deg2rad, '-b'), grid, title('Fused roll');
    subplot(2, 2, 4), plot((rpys(2, :)) / deg2rad, '-b'), grid, title('Fused pitch');

    tgt_enu = zeros(3, size(raw_xyz, 2));
    for i = 1:size(raw_xyz, 2)
        raw_enu(:, i) = rpy2dcm(src_rpy(1:3, i)) * raw_xyz(:, i);
    end

    figure(4);
    subplot(3, 1, 1), plot(raw_enu(3, :)), grid, title('Raw U');
    subplot(3, 1, 2), plot(-tgt_ned(3, :)), grid, title('Fused D (negated)');
    subplot(3, 1, 3), plot(tgt_geod(3, :)), grid, title('Fused Alt');

    dist = sqrt(raw_xyz(1, :).^2 + raw_xyz(2, :).^2 + raw_xyz(3, :).^2);
    brng = atan2(raw_xyz(2, :), raw_xyz(1, :));
    elev = asin(raw_xyz(3, :) ./ dist);

    figure(5);
    subplot(2, 2, 1), plot(unwrap(rpys(3, :)) / deg2rad), grid, title('Fused Yaw');
    subplot(2, 2, 2), plot(unwrap(brng) / deg2rad), grid, title('Raw Bearing');
    subplot(2, 2, 3), plot(unwrap(-src_rpy(3, :)) / deg2rad), grid, title('Raw Yaw (negated)');
    subplot(2, 2, 4), plot(unwrap(elev) / deg2rad), grid, title('Raw Elevation');

    figure(6);
    plot3(raw_enu(1, :), raw_enu(2, :), raw_enu(3, :), '.k'), grid, title('Raw ENU');
end

function plot_target(tgt, src, ned)
    k = wmerc2scale(src(:, 1));

    xt = (tgt(1, :) - src(1, 1)) / k;
    yt = (tgt(2, :) - src(2, 1)) / k;
    zt = tgt(3, :);

    xs = (src(1, :) - src(1, 1)) / k;
    ys = (src(2, :) - src(2, 1)) / k;
    zs = src(3, :);

    xn = -ned(2, :) + (tgt(1, 1) - src(1, 1)) / k;
    yn = -ned(1, :) + (tgt(2, 1) - src(2, 1)) / k;
    zn = ned(3, :) + tgt(3, 1);

    t = 1:size(xt, 2);

    disp('STD [x,y]:');
    disp([ std(xt), std(yt) ]);

    figure(1), hold('off');
    plot3(xs, ys, zs, '.k');
    hold('on'), grid('on');
    plot3(xt, yt, zt, '.r');
    plot3(xn, yn, zn, '.b');
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
