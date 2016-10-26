%
% USBL calibration
%
% Input: src -- array of CRP coordinates (ECEF)
%        sdcm -- array of corresponding LF rotations (NED)
%        xyz -- array of corresponding measured target coordinates in LF
function [ dcm, shift, ntri ] = usbl_calib(src, sdcm, xyz, talt)
    dcm = [];
    shift = [];
    ntri = 0;

    tgt = [];
    tri = [];

    flip = [ 0, 1, 0; 1, 0, 0; 0, 0, -1 ];
    step = floor(size(src, 2) / 15);
    for i = 1:step
        decim = i:step:size(src, 2);
        [ tgt_ecef, tri_ecef ] = slbl(src(:, decim), sqrt(sum(xyz(:, decim).^2, 1)));

        if size(tgt_ecef, 1) ~= 0
            tgt = [ tgt, tgt_ecef ];
            tri = [ tri, tri_ecef ];
        end
    end

    ntri = size(tgt, 2);
    if ntri ~= 0
        % plot_slbl(tgt);
        % plot_tri(src, tgt, reshape(tri, 3, 3, size(tri, 2) / 3));

        etgt = mean(tgt, 2);
        if nargin > 3 % target altitude is not defined
            gtgt = ecef2geod(etgt);
            gtgt(3) = talt;
            etgt = geod2ecef(gtgt);
        end

        meas = flip * xyz;
        real = ecef_to_local(etgt, src, sdcm);

        [ dcm, shift ] = find_transform(real, meas);
        % plot_calib(real, meas, dcm * meas + shift);
    end
end

%
% Compute target coordinates to Local Frame for each source position
%
% Input: tgt_ecef -- target coordinates (one point)
%        src_ecef -- array of source coordinates
%        src_dcm -- array of DCMs (LF -> NED) for each source point
%
% Output: tgt_xyz -- coordinates of tgt_ecef in local frame
function tgt_xyz = ecef_to_local(tgt_ecef, src_ecef, src_dcm)
    enu2ned = [ 0, 1, 0; 1, 0, 0; 0, 0, -1 ];
    tgt_xyz = [];

    n = size(src_ecef, 2);
    if n == 0
        return;
    end

    ecef_dcm = geod2dcm(ecef2geod(src_ecef));

    for i = 1:n
        tgt_xyz = [ tgt_xyz, src_dcm(:, :, i)' * enu2ned * ecef_dcm(:, :, i)' * (tgt_ecef - src_ecef(:, i)) ];
    end
end

function plot_tri(src, tgt, tri)
    figure(5), hold('off');
    src_wm = geod2wmerc(ecef2geod(src));
    tgt_wm = geod2wmerc(ecef2geod(tgt));
    o = src_wm(:, 1);
    k = wmerc2scale(o);

    plot3((src_wm(1, :) - o(1)) / k, (src_wm(2, :) - o(2)) / k, src_wm(3, :), '.k');
    hold('on'), grid('on');
    plot3((tgt_wm(1, :) - o(1)) / k, (tgt_wm(2, :) - o(2)) / k, tgt_wm(3, :), '.r');

    n = size(tri, 3);
    for i = 1:n
        t = geod2wmerc(ecef2geod(tri(:, :, i)));
        t(1:2, :) = (t(1:2, :) - o(1:2)) / k;
        t = [ t, t(:, 1) ];

        plot3(t(1, :), t(2, :), t(3, :), '-');
    end
end

function plot_slbl(tgt_ecef)
    wm = geod2wmerc(ecef2geod(tgt_ecef));
    k = wmerc2scale(wm(:, 1));

    x = (wm(1, :) - wm(1, 1)) * 100 / k;
    y = (wm(2, :) - wm(2, 1)) * 100 / k;

    figure(3), title('SLBL');
    subplot(2, 1, 1), plot(x, y, '.-'), grid('on'), title('SLBL Lat/Lon [cm]');
    subplot(2, 1, 2), plot(wm(3, :), '.-'), title('SLBL Altitude [m]');
end

function plot_calib(real, meas, corr)
    figure(2), hold('off'), title('USBL-frame target coordinates');
    plot3(real(1, :), real(2, :), -real(3, :), 'k.');
    hold('on'), grid('on');
    plot3(corr(1, :), corr(2, :), -corr(3, :), 'rx');
    plot3(meas(1, :), meas(2, :), -meas(3, :), 'b.');
    legend('real', 'corr', 'meas');
end
