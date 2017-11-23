function [rs] = test_ahrs(rpy)
    d2r = pi / 180;
    flip = [ 0, 1, 0; 1, 0, 0; 0, 0, -1 ];
    enu = rpy2dcm(rpy' .* d2r);

    %ned = flip * enu * flip; % X'=Y, Y'=Z, Z'=-Z
    %ned = flip * enu * flip * rpy2dcm([0; 0; pi/2]); % X'=X, Y'=-Y, Z'=-Z

    % 1: X'=Y, Y'=Z, Z'=-Z
    % 2: sensor rotated on -90deg
    ned = flip * enu * flip * rpy2dcm([0; 0; -pi/2])';

    rs = dcm2rpy(ned)' ./ d2r;
end
