%
% Fuse USBL, AHRS and GNSS measurements
%
% Input: tgt_usbl_xyz -- target coorsinates measured by USBL
%        src_ahrs_rpy -- source Euler angles measured ny AHRS (to NED)
%        src_geod -- geodetic coordinates measured by GNSS
%
% To convert RPY "to ENU" to "to NED":
%       rpy_to_ned = dcm2rpy(flip * rpy2dcm(rpy_to_enu));
% where flip == enu2ned == ned2enu == rpy2dcm([ pi; 0; pi/2 ])
%
function [ tgt ] = usbl_defuse(tgt, src_ahrs_rpy, is_enu, src_geod)
    global usbl_dev_xyz; % shift in local frame of USBL relative to CRP
    global usbl_dev_dcm; % rotation of USBL in local frame
    global ahrs_dev_dcm; % rotation of AHRS in local frame

    flip = [ 0, 1, 0; 1, 0, 0; 0, 0, -1 ];
    n = size(src_ahrs_rpy, 2);

    if nargin > 3
        tgt = geod2ecef(tgt); % ECEF

        src_ecef = geod2ecef(src_geod);
        ecef_dcm = geod2dcm(src_geod);

    end

    if size(src_ahrs_rpy, 1) > 3
        arpy = dcm2rpy(ahrs_dev_dcm);
        adcm_y = rpy2dcm([ 0; 0; arpy(3) ]);
    end

    for i = 1:n
        rpy = src_ahrs_rpy(1:3, i);
        if size(src_ahrs_rpy, 1) == 4
            rpy(3) = 0;
        end

        dcm = rpy2dcm(rpy);
        if is_enu ~= 0
            dcm = ned2enu * dcm * ned2enu;
        end

        if size(src_ahrs_rpy, 1) == 4
            dcm_y = rpy2dcm([ 0; 0; src_ahrs_rpy(4, i) ]);
            dcm = dcm_y * adcm_y * dcm;
        end

        src_dcm = dcm * ahrs_dev_dcm';

        if nargin > 3
            tgt(:, i) = flip * ecef_dcm(:, :, i)' * (tgt(:, i) - src_ecef(:, i)); % LF NED
        end

        tgt(:, i) = src_dcm' * tgt(:, i); % LF XYZ
    end

    tgt = flip * usbl_dev_dcm' * (tgt - usbl_dev_xyz); % USBL XYZ
end
