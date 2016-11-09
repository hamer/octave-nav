function n = days_between(date1, date2)
    n = day_in_year(date2) - day_in_year(date1);

    y1 = date1(3); y2 = date2(3);
    for y = y1:y2-1
        if is_leap_year(y)
            n = n + 366;
        else
            n = n + 365;
        end
    end
end
