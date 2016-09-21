function ecef = geod2ecef(geod)
    global Sa;
    global Sb;

    deg2rad = pi / 180;
    cp = cos(geod(1) * deg2rad); cl = cos(geod(2) * deg2rad);
    sp = sin(geod(1) * deg2rad); sl = sin(geod(2) * deg2rad);
    n = get_n(geod(1) * deg2rad);
    r = (n + geod(3)) * cp;

    ecef = [ r * cl; r * sl; ((Sb / Sa)^2 * n + geod(3)) * sp ];
end
