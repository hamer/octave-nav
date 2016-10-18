function k = geod2scale(geod)
    k = sec(geod(1, :) * pi / 180);
end
