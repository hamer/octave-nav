function wgs84()
    global Sa;
    global Sb;
    global Sf;
    global Se;

    Sa = 6378137;
    Sb = 6356752.3142;
    Sf = 1 / 298.257223563;    % f = 1 - b/a
    Se = 0.0818191909289069;   % e = sqrt(1 - (b/a)^2)
end
