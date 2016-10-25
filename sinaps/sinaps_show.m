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

    if nargin < 5 || strcmp(arot, 'fused')
        arot = [ 0, 0, 0 ];
    end

    if nargin < 6
        ehdt_flag = 0;
    end

    usbl_dev_xyz = ushift';
    usbl_dev_dcm = rpy2dcm(urot' * deg2rad);
    ahrs_dev_dcm = rpy2dcm(arot' * deg2rad);

    data = csvread(name, 0, 1);
    data = data(find(data(:, 37) ~= 0), :);     % filter out records without gps
    data = data(find(data(:, 3) == addr), :);   % filter out records with wrong address
    nlines = size(data, 1);

    raw_xyz = data(:, 10:12)';      % XYZ coordinates of a target in USBL frame
    raw_rpy = data(:, 26:28)';      % raw Roll/Pitch/Yaw
    ehdt = data(:, 31)';            % vessels Yaw
    crp_geod = data(:, 37:39)';     % Lat, Lon and Alt of CRP

    if strcmp(arot, 'fused')
        src_rpy = data(:, 29:31)';  % vessels Roll/Pitch/Yaw
    elseif ehdt_flag == 1
        src_rpy = [ raw_rpy; ehdt ];
    else
        src_rpy = raw_rpy;
    end

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
