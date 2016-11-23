function s = smod(x, m)
    s = x - round(x ./ m) .* m;
end
