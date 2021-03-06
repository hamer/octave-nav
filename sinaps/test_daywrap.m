function test_daywrap(date1, time1, time2)
    t1 = utc2timestamp(time1, date1);
    t2 = utc2timestamp(time2);

    diff = wrap_daydiff(t2 - t1);
    [ rtime, rdate ] = timestamp2utc(t1 + diff);

    disp('Difference [days]:'), disp(round(diff / 86400));
    disp('Difference [seconds]:'), disp(diff);
    disp('Date:'), disp(rdate);
    disp('Time:'), disp(rtime);
end
