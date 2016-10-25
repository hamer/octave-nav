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
function [ tgt_usbl_xyz ] = usbl_defuse(tgt_ecef, src_ahrs_rpy, src_geod)
    global usbl_dev_xyz; % shift in local frame of USBL relative to GNSS
    global usbl_dev_dcm; % rotation of USBL in local frame
    global ahrs_dev_dcm; % rotation of AHRS in local frame

    enu2ned = [ 0, 1, 0; 1, 0, 0; 0, 0, -1 ];

    if length(src_ahrs_rpy) == 3 % [ roll, pitch, heading ]
        src_dcm = rpy2dcm(src_ahrs_rpy) * ahrs_dev_dcm';
    else % [ roll, pitch, _, true_heading ]
        dcm = rpy2dcm(src_ahrs_rpy) * ahrs_dev_dcm';
        rpy = dcm2rpy(dcm);
        src_dcm = rpy2dcm([ 0; 0; src_ahrs_rpy(4) - rpy(3) ]) * dcm;
    end

    src_ecef = geod2ecef(src_geod);
    ecef_dcm = geod2dcm(src_geod);

    tgt_ned = enu2ned * ecef_dcm' * (tgt_ecef - src_ecef);
    tgt_xyz = src_dcm' * tgt_ned;

    tgt_usbl_xyz = usbl_dev_dcm' * (tgt_xyz - usbl_dev_xyz);
end
