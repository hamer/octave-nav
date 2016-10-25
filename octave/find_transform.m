function [ dcm, shift ] = find_transform(dst, src)
    csrc = mean(src, 2);
    cdst = mean(dst, 2);

    dsrc = src - csrc;
    ddst = dst - cdst;

    n = zeros(3, 3);
    for i = 1:size(dst, 2)
        n = n + dsrc(:, i) * ddst(:, i)';
    end

    [ u, s, v ] = svd(n);
    dcm = v * u';
    shift = cdst - dcm * csrc;
end
