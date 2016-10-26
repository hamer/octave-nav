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
function [ tgt ] = usbl_defuse(tgt_geod, src_ahrs_rpy, src_geod)
    global usbl_dev_xyz; % shift in local frame of USBL relative to CRP
    global usbl_dev_dcm; % rotation of USBL in local frame
    global ahrs_dev_dcm; % rotation of AHRS in local frame

    enu2ned = [ 0, 1, 0; 1, 0, 0; 0, 0, -1 ];

    n = size(src_ahrs_rpy, 2);
    tgt = geod2ecef(tgt_geod); % ECEF

    src_ecef = geod2ecef(src_geod);
    ecef_dcm = geod2dcm(src_geod);

    if size(src_ahrs_rpy, 1) > 3
        arpy = dcm2rpy(ahrs_dev_dcm);
        adcm_y = rpy2dcm([ 0; 0; arpy(3) ]);
    end

    for i = 1:n
        if size(src_ahrs_rpy, 1) == 3 % [ roll, pitch, heading ]
            src_dcm = rpy2dcm(src_ahrs_rpy(:, i)) * ahrs_dev_dcm';
        else % [ roll, pitch, _, true_heading ]
            dcm_rp = rpy2dcm([ src_ahrs_rpy(1:2, i); 0 ]);
            dcm_y = rpy2dcm([ 0; 0; src_ahrs_rpy(4, i) ]);

            src_dcm = dcm_y * adcm_y * dcm_rp * ahrs_dev_dcm';
        end

        tgt(:, i) = enu2ned * ecef_dcm(:, :, i)' * (tgt(:, i) - src_ecef(:, i)); % LF NED
        tgt(:, i) = src_dcm' * tgt(:, i); % LF XYZ
    end

    tgt = usbl_dev_dcm' * (tgt - usbl_dev_xyz); % USBL XYZ
end
