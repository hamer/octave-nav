function geod = ecef2geod(ecef)
    global Se;
    global Sb;

    p = sqrt(ecef(1)^2 + ecef(2)^2);
    phi = atan2(ecef(3), p * (1 - Se^2));

    if p < 0.01
        h = ecef(3) - Sb * sin(phi);
    else
        h = 0;
        i = 0;
        while i < 10
            h_prev = h;

            n = get_n(phi);
            h = p / cos(phi) - n;
            phi = atan2(ecef(3), p * (1 - Se^2 * n / (n + h)));

            if abs(h_prev - h) <= 0.01
                break;
            end

            i = i + 1;
        end
    end

    rad2deg = 180 / pi;
    geod = [ phi * rad2deg; atan2(ecef(2), ecef(1)) * rad2deg; h ];
end
