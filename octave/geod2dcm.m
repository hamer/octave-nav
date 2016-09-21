%
% Topocentric basis (ENU) in ECEF
%
function dcm = geod2dcm(geod)
    deg2rad = pi / 180;
    cp = cos(geod(1) * deg2rad); cl = cos(geod(2) * deg2rad);
    sp = sin(geod(1) * deg2rad); sl = sin(geod(2) * deg2rad);

    % dcm = rpy2dcm([ pi / 2 - geod(1) * deg2rad, 0, pi / 2 + geod(2) * deg2rad ]);
    dcm = [ -sl, -sp * cl, cp * cl;
             cl, -sp * sl, cp * sl;
              0,       cp,      sp ];
end
