function test_daywrap(h1, m1, s1, h2, m2, s2)
    t1 = s1 + 60 * m1 + 3600 * h1;
    t2 = s2 + 60 * m2 + 3600 * h2;

    diff = wrap_daydiff(t2 - t1);
    t3 = wrap_day(t1 + diff);

    h3 = floor(t3 / 3600);
    m3 = floor(t3 / 60 - h3 * 60);
    s3 = floor(t3 - h3 * 3600 - m3 * 60);

    disp('Difference [days]:'), disp(floor(diff / 86400 + 0.5));
    disp('T1:'), disp([ h1, m1, s1 ]);
    disp('T2:'), disp([ h2, m2, s2 ]);
    disp('T3:'), disp([ h3, m3, s3 ]);
end
