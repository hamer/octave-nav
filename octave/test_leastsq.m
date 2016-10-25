function test_leastsq()
    deg2rad = pi / 180;
    i = 1:100;
    pts = [ 1.0 * cos(i / 100 * (2 * pi)); ...
            0.4 * sin(i / 100 * (2 * pi)); ...
            1.0 * sin(i / 100 * (3 * pi)) ];

    % TODO: find convergence relation to figure scale and distance from origin
    src = 10 * pts + [ 0; 0; 10 ];

    sxyz = [ 1, 2, 6 ]
    srpy = [ 1, 2, 124 ]
    namp = 0;

    noise = 2 * namp * (rand(size(src)) - 0.5);
    dst = sxyz' + rpy2dcm(srpy' * deg2rad) * src + noise;

    [ dcm, shift ] = find_transform(dst, src);

    %% answer
    txyz = shift'
    trpy = dcm2rpy(dcm)' / deg2rad
end
