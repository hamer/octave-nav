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
function sinaps_show(name, addr, urot, ushift, arot, ehdt_flag)
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
    elseif nargin < 6|| ehdt_flag ~= 1
        src_rpy = data(:, 26:28)';  % raw Roll/Pitch/Yaw
    else
        src_rpy = data(:, [ 26:28, 31 ])';  % raw Roll/Pitch/Yaw + vessel Yaw
    end

    enu2ned = [ 0, 1, 0; 1, 0, 0; 0, 0, -1 ];

    for i = 1:size(src_rpy, 2)
        % convert USBL-RPY to LF-RPY (only for internal AHRS)
        src_rpy(1:3, i) = dcm2rpy(enu2ned * rpy2dcm(src_rpy(1:3, i)) * enu2ned);
    end

    if ehdt_flag == 1
        dyaw = src_rpy(4, :) - src_rpy(3, :);
        figure(4), plot((dyaw - mean(dyaw)) / deg2rad, '-b'), grid('on'), title('Yaw difference');
    end

    [ tgt_ecef, dcms ] = usbl_fuse(enu2ned * raw_xyz, src_rpy, crp_geod);
    plot_target(geod2wmerc(ecef2geod(tgt_ecef)), geod2wmerc(crp_geod));

    rpys = dcm2rpy(dcms);
    figure(5);
    subplot(2, 1, 1), plot((rpys(1, :)) / deg2rad, '-b'), grid('on'), title('Fused roll');
    subplot(2, 1, 2), plot((rpys(2, :)) / deg2rad, '-b'), grid('on'), title('Fused pitch');
end

function plot_target(tgt, src)
    k = wmerc2scale(src(:, 1));

    xt = (tgt(1, :) - src(1, 1)) / k;
    yt = (tgt(2, :) - src(2, 1)) / k;
    zt = tgt(3, :);

    xs = (src(1, :) - src(1, 1)) / k;
    ys = (src(2, :) - src(2, 1)) / k;
    zs = src(3, :);

    figure(1), hold('off');
    plot3(xs, ys, zs, '.k');
    hold('on'), grid('on');
    plot3(xt, yt, zt, '.r');
    legend('CRP', 'Target'), title('Target');
end
