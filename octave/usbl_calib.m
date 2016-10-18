%
% USBL calibration
%
% Input: src -- array of CRP coordinates (ECEF)
%        quat -- array of corresponding LF (NED)
%        xyz -- array of corresponding measured target coordinates in LF
function [ dcm, shift, iter ] = usbl_calib(src, quat, xyz)
    real = [];
    meas = [];

    step = floor(size(src, 2) / 15);
    for i = 1:step
        decim = i:step:size(src, 2);

        src_ecef = src(:, decim);
        tgt_xyz = xyz(:, decim);
        tgt_ecef = slbl(src_ecef, sqrt(sum(tgt_xyz.^2, 1)));

        if size(tgt_ecef, 1) == 0
            continue;
        end

        real = [ real, ecef_to_local(mean(tgt_ecef, 2), src_ecef, quat(:, decim)) ];
        meas = [ meas, tgt_xyz ];
    end

    [ dcm, shift, iter ] = find_transform(real, meas);
    disp('Iterations during calibration:'), disp(iter);

    % if size(meas, 1) ~= 0
    %     plot_calib(real, meas, dcm * meas + shift);
    % end
end

%
% Compute target coordinates to Local Frame for each source position
%
% Input: tgt_ecef -- target coordinates (one point)
%        src_ecef -- array of source coordinates
%        quat -- array of quaternions (LF -> NED) for each source point
%
% Output: tgt_xyz -- coordinates of tgt_ecef in local frame
function tgt_xyz = ecef_to_local(tgt_ecef, src_ecef, src_quat)
    enu2ned = [ 0, 1, 0; 1, 0, 0; 0, 0, -1 ];
    tgt_xyz = [];

    n = size(src_ecef, 2);
    if n == 0
        return;
    end

    for i = 1:n
        ecef_dcm = geod2dcm(ecef2geod(src_ecef(:, i)));
        src_dcm = quat2dcm(src_quat(:, i));

        tgt_xyz = [ tgt_xyz, src_dcm' * enu2ned * ecef_dcm' * (tgt_ecef - src_ecef(:, i)) ];
    end
end

function plot_calib(real, meas, corr)
    figure(2), hold('off');
    plot3(real(1, :), real(2, :), -real(3, :), 'k.');
    hold('on'), grid('on');
    plot3(corr(1, :), corr(2, :), -corr(3, :), 'rx');
    plot3(meas(1, :), meas(2, :), -meas(3, :), 'b.');
    legend('real', 'meas', 'corr');
end
