function wmerc = geod2wmerc(geod)
    global Sa;

    deg2rad = pi / 180;
    wmerc = [ Sa * geod(2, :) * deg2rad;
              Sa * log(tan(pi / 4 + geod(1, :) * deg2rad / 2));
              geod(3, :) ];
end
