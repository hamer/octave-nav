function gnss_round()
    center = [ 50.0; 13.0; 45.0 ];
    hdg = 0:1:359;

    wgs84();

    vec = [ cos(hdg .* pi ./ 180); sin(hdg .* pi ./ 180) ];

    for i = 1:size(hdg, 2)
        nvec = [100 * vec(:, i); 0];
        ng = ned2geod(center, nvec);
        uvec = rpy2dcm([0; 0; -hdg(i) - 90] * pi / 180) * (nvec);

        printf('a %5d  %+7.1f %+7.1f %+7.1f\n', i * 10, 0, 0, hdg(i) - 90.0);
        printf('g %5d  %+12.7f %+12.7f %+7.2f\n', i * 10, ng(1), ng(2), ng(3));
        printf('u %5d  %+7.2f %+7.2f %+7.2f 0\n', i * 10, uvec(1), uvec(2), uvec(3));
    end
end

function [rs] = ned2geod(src_geod, ned)
    flip = [ 0, 1, 0; 1, 0, 0; 0, 0, -1 ];
    src_ecef = geod2ecef(src_geod);
    ecef_dcm = geod2dcm(src_geod);
    tgt_ecef = src_ecef + ecef_dcm * flip * ned;
    rs = ecef2geod(tgt_ecef);
end
