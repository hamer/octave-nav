function uts = utc2timestamp(utc_time, utc_date)
    utc_epoch = [ 1, 1, 1970 ];

    if nargin < 2
        utc_date = utc_epoch;
    end

    hour   = utc_time(1);
    minute = utc_time(2);
    second = utc_time(3);

    uts = days_between(utc_epoch, utc_date) * 86400;
    uts = uts + hour * 3600 + minute * 60 + second;
end
