function [ dcm, shift ] = find_transform(real, meas)
    cmeas = mean(meas, 2);
    creal = mean(real, 2);

    dmeas = meas - cmeas;
    dreal = real - creal;

    n = zeros(3, 3);
    for i = 1:size(real, 2)
        n = n + dmeas(:, i) * dreal(:, i)';
    end

    [ u, s, v ] = svd(n);
    dcm = v * u';
    shift = creal - dcm * cmeas;
end
