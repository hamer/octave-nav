function rpy = dcm2rpy(dcm)
    rpy = [ atan2(dcm(3, 2), dcm(3, 3));
            asin(-dcm(3, 1));
            atan2(dcm(2, 1), dcm(1, 1)) ];
end
