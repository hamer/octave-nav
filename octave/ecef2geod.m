function geod = ecef2geod(ecef)
    global Se;
    global Sa;
    global Sb;

    p = sqrt(ecef(1)^2 + ecef(2)^2);

    %% iterative solution
    % phi = atan2(ecef(3), p * (1 - Se^2));

    % if p < 0.01
    %     h = ecef(3) * sin(phi) - Sb;
    % else
    %     h = 0;
    %     i = 0;
    %     while i < 10
    %         h_prev = h;

    %         n = get_n(phi);
    %         h = p / cos(phi) - n;
    %         phi = atan2(ecef(3), p * (1 - Se^2 * n / (n + h)));

    %         if abs(h_prev - h) <= 0.01
    %             break;
    %         end

    %         i = i + 1;
    %     end
    % end

    %% non-iterative solution
    Se2 = Se^2 / (1 - Se^2);

    q = atan2(ecef(3) * Sa, p * Sb);
    phi = atan2(ecef(3) + Se2 * Sb * sin(q)^3, p - Sa * Se^2 * cos(q)^3);
    h = p / cos(phi) - get_n(phi);

    rad2deg = 180 / pi;
    geod = [ phi * rad2deg; atan2(ecef(2), ecef(1)) * rad2deg; h ];
end
