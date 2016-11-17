function [ utc_time, utc_date ] = timestamp2utc(uts)
    day = 86400;

    time = mod(uts, day);
    date = (uts - time) / day;

    acc = 0;
    year = 1970;
    ndays = 365;
    while date >= acc + ndays;
        acc = acc + ndays;
        year = year + 1;
        ndays = 365 + is_leap_year(year);
    end

    md = [ 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 ];

    if is_leap_year(year)
        md(2) = 29;
    end

    date = date - acc;

    acc = 0;
    month = 1;
    while date >= acc + md(month)
        acc = acc + md(month);
        month = month + 1;
    end

    date = date - acc;
    day = date + 1;

    hour   = floor(time / 3600);
    minute = floor(time / 60 - hour * 60);
    second = floor(time - hour * 3600 - minute * 60);

    utc_time = [ hour, minute, second ];
    utc_date = [ day, month, year ];
end
