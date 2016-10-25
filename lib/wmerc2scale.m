function k = wmerc2scale(wmerc)
    global Sa;
    k = cosh(wmerc(2, :) / Sa);
end
