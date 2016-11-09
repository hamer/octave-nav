function n = day_in_year(date)
    md = [ 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 ];
    d = date(1); m = date(2); y = date(3);

    if is_leap_year(y)
        md(2) = 29;
    end

    n = d + sum(md(1:m-1));
end
