function geod = wmerc2geod(wmerc)
    global Sa;

    rad2deg = 180 / pi;
    geod = [ 90 - 2 * atan(exp(-wmerc(2, :) / Sa)) * rad2deg;
             wmerc(1, :) / Sa * rad2deg;
             wmerc(3, :) ];
end
