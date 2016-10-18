function [ dcm, shift, iter ] = find_transform(real, meas)
    dcm = eye(3);
    shift = zeros(3, 1);

    iter = 0;
    eps = 1e-3;

    if size(real, 1) == 0
        return;
    end

    while iter < 10000
        [ cdcm, cxyz ] = find_single_transform(real, shift + dcm * meas);
        shift = cdcm * shift + cxyz;
        dcm = cdcm * dcm;
        iter = iter + 1;

        if norm(cxyz) < eps && norm(cdcm - eye(3)) < eps^2
            break;
        end
    end
end

function [ dcm, shift ] = find_single_transform(real, meas)
    cmeas = mean(meas, 2);
    creal = mean(real, 2);

    dmeas = meas - cmeas;
    dreal = real - creal;

    n = zeros(3, 3);
    for i = 1:size(real, 2)
        n = n + meas(:, i) * real(:, i)';
    end

    [ u, s, v ] = svd(n);
    dcm = v * u';
    shift = creal - dcm * cmeas;
end
