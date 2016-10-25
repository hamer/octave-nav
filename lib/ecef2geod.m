function geod = ecef2geod(ecef)
    global Se;
    global Sa;
    global Sb;

    p = sqrt(ecef(1, :).^2 + ecef(2, :).^2);

    Se2 = Se^2 / (1 - Se^2);

    q = atan2(ecef(3, :) * Sa, p * Sb);
    phi = atan2(ecef(3, :) + Se2 * Sb * sin(q).^3, p - Sa * Se^2 * cos(q).^3);
    h = p ./ cos(phi) - get_n(phi);

    rad2deg = 180 / pi;
    geod = [ phi * rad2deg; atan2(ecef(2, :), ecef(1, :)) * rad2deg; h ];
end
