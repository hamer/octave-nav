function raw_fuse()
    global usbl_dev_xyz; % shift in local frame of USBL relative to CRP
    global usbl_dev_dcm; % rotation of USBL in local frame
    global ahrs_dev_dcm; % rotation of AHRS in local frame

    deg2rad = pi / 180;
    wgs84();

    usbl_dev_xyz = [ -15; 0; 10 ];
    usbl_dev_dcm = rpy2dcm([ 0; 0; -32 ] * deg2rad);
    ahrs_dev_dcm = rpy2dcm([ 0; 0; 0 ] * deg2rad);

    % read data from files
    raw_usbl = read_usbl();
    raw_ahrs = read_ahrs();
    raw_gnss = read_gnss();

    szu = size(raw_usbl, 1);
    tgt_xyz = raw_usbl(:, 4:6);
    src_rpy = zeros(size(tgt_xyz));
    src_geod = zeros(size(tgt_xyz));

    for iu = 1:szu
        t = raw_usbl(iu, 1) - raw_usbl(iu, 3);

        % select last AHRS measurement before USBL
        ia = find(raw_ahrs(:, 1) <= t, 1, 'last');
        if ~isempty(ia)
            src_rpy(iu, :) = raw_ahrs(ia, 2:4);
            raw_ahrs = raw_ahrs(ia:end, :);
        end

        % select last GNSS measurement before USBL
        ig = find(raw_gnss(:, 1) <= t, 1, 'last');
        if ~isempty(ig)
            src_geod(iu, :) = raw_gnss(ig, [ 3, 4, 6 ]);
            raw_gnss = raw_gnss(ig:end, :);
        end
    end

    idx = find(src_geod(:, 1) ~= 0);
    idx = idx(1800:end); % remove 15.12.2016 maneuers
    idx = idx(13000:17000); % select square maneuer

    tgt_xyz = tgt_xyz(idx, :)';
    src_rpy = src_rpy(idx, :)' * deg2rad;
    src_geod = src_geod(idx, :)';

    tgt_ned = usbl_fuse(tgt_xyz, src_rpy, 0);
    tgt_geod = usbl_fuse(tgt_xyz, src_rpy, 0, src_geod);

    plot_target(geod2wmerc(tgt_geod), geod2wmerc(src_geod), tgt_ned);
end

function [ raw ] = read_usbl()
    raw = csvread('raw/usbl.csv');

    raw(:, 3) = raw(:, 2) - raw(:, 2); % delay (stime - ftime)

    % normalize sound velocity to 1500
    mul = 1500e-6 * raw(:, 7) ./ sqrt(raw(:, 4).^2 + raw(:, 5).^2 + raw(:, 6) .^2);
    raw(:, 4) = raw(:, 4) .* mul;
    raw(:, 5) = raw(:, 5) .* mul;
    raw(:, 6) = raw(:, 6) .* mul;
end

function [ raw ] = read_ahrs()
    raw = csvread('raw/ahrs.csv');
end

function [ raw ] = read_gnss()
    raw = csvread('raw/gnss.csv');
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
