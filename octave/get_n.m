%
% curvature radius of ellipsoid on specific latitude
%
function n = get_n(phi)
    global Sa;
    global Se;

    n = Sa / sqrt(1 - (Se * sin(phi))^2);
end
