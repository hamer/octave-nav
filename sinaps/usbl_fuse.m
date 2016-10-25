%
% Fuse USBL, AHRS and GNSS measurements
%
% Input: tgt_usbl_xyz -- target coorsinates measured by USBL
%        src_ahrs_rpy -- source Euler angles measured ny AHRS (to NED)
%        src_geod -- geodetic coordinates measured by GNSS
%
% To convert RPY "to ENU" to "to NED":
%       rpy_to_ned = dcm2rpy(enu2ned * rpy2dcm(rpy_to_enu));
% where enu2ned == ned2enu == rpy2dcm([ pi; 0; pi/2 ])
%
function [ rs, src_dcm, src_ecef ] = usbl_fuse(tgt_usbl_xyz, src_ahrs_rpy, src_geod)
    global usbl_dev_xyz; % shift in local frame of USBL relative to CRP
    global usbl_dev_dcm; % rotation of USBL in local frame
    global ahrs_dev_dcm; % rotation of AHRS in local frame

    ned2enu = [ 0, 1, 0; 1, 0, 0; 0, 0, -1 ];

    tgt_xyz = usbl_dev_xyz + usbl_dev_dcm * tgt_usbl_xyz;

    if nargin < 2
        rs = tgt_xyz;
        src_dcm = [];
        src_ecef = [];
        return;
    end

    n = size(src_ahrs_rpy, 2);
    src_dcm = zeros(3, 3, n);
    tgt = zeros(3, n);

    for i = 1:n
        if size(src_ahrs_rpy, 1) == 3 % [ roll, pitch, heading ]
            src_dcm(:, :, i) = rpy2dcm(src_ahrs_rpy(:, i)) * ahrs_dev_dcm';
        else % [ roll, pitch, _, true_heading ]
            dcm = rpy2dcm(src_ahrs_rpy(:, i)) * ahrs_dev_dcm';
            rpy = dcm2rpy(dcm);
            src_dcm(:, :, i) = rpy2dcm([ 0; 0; src_ahrs_rpy(4, i) - rpy(3) ]) * dcm;
        end

        tgt(:, i) = src_dcm(:, :, i) * tgt_xyz(:, i);
    end

    if nargin < 3
        rs = tgt;
        src_ecef = [];
        return;
    end

    src_ecef = geod2ecef(src_geod);
    ecef_dcm = geod2dcm(src_geod);

    for i = 1:n
        tgt(:, i) = src_ecef(:, i) + ecef_dcm(:, :, i) * ned2enu * tgt(:, i);
    end

    rs = tgt;
end
