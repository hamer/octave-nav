function rs = is_leap_year(year)
    rs = mod(year, 400) == 0 || (mod(year, 4) == 0 && mod(year, 100) ~= 0);
end
