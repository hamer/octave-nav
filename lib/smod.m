function s = smod(x, m)
    s = x - floor(x ./ m + 0.5) .* m;
end
