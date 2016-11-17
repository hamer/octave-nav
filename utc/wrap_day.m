function ao = wrap_day(ai)
    day = 86400;
    ao = smod(ai - day / 2, day) + day / 2;
end
