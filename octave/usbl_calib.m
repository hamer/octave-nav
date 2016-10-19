%
% USBL calibration
%
% Input: src -- array of CRP coordinates (ECEF)
%        sdcm -- array of corresponding LF rotations (NED)
%        xyz -- array of corresponding measured target coordinates in LF
function [ dcm, shift, iter ] = usbl_calib(src, sdcm, xyz)
    real = [];
    meas = [];

    step = floor(size(src, 2) / 15);
    for i = 1:step
        decim = i:step:size(src, 2);

        src_ecef = src(:, decim);
        tgt_xyz = xyz(:, decim);
        [ tgt_ecef, tri_ecef ] = slbl(src_ecef, sqrt(sum(tgt_xyz.^2, 1)));

        if size(tgt_ecef, 1) == 0
            continue;
        end

        % plot_slbl(tgt_ecef);
        % plot_tri(src, tgt_ecef, reshape(tri_ecef, 3, 3, size(tri_ecef, 2) / 3));

        real = [ real, ecef_to_local(mean(tgt_ecef, 2), src_ecef, sdcm(:, :, decim)) ];
        meas = [ meas, tgt_xyz ];
    end

    [ dcm, shift ] = find_transform(real, meas);

    % if size(meas, 1) ~= 0
    %     plot_calib(real, meas, dcm * meas + shift);
    % end
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
    figure(4), hold('off');
    src_wm = geod2wmerc(ecef2geod(src));
    tgt_wm = geod2wmerc(ecef2geod(tgt));
    o = src_wm(:, 1);
    k = wmerc2scale(o);

    plot((src_wm(1, :) - o(1)) / k, (src_wm(2, :) - o(2)) / k, '.k');
    hold('on'), grid('on');
    plot((tgt_wm(1, :) - o(1)) / k, (tgt_wm(2, :) - o(2)) / k, '.r');

    n = size(tri, 3);
    for i = 1:n
        t = (geod2wmerc(ecef2geod(tri(:, :, i))) - o) / k;
        t = [ t, t(:, 1) ];

        plot(t(1, :), t(2, :), '-');
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
