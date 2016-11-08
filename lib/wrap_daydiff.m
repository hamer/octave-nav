function so = wrap_daydiff(si)
    day = 86400;
    so = smod(si, 2 * day);
end
