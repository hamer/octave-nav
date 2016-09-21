%
% Syntethic LBL (ECEF)
%
% Input: src_ecef -- array of source coordinates
%        dist -- array of measured distances to target from corresponding soucre
%
% Output: tgt_ecef -- array of valid computed points
function tgt_ecef = slbl(src_ecef, dist)
    tgt_ecef = [];

    tv = triangulate(src_ecef);
    n = size(tv, 1);
    if n == 0
        return;
    end

    for i = 1:n
        crd = src_ecef(:, tv(i, :));
        r = dist(tv(i, :));

        [ d1, d2, valid ] = trilat(crd(:, 1), crd(:, 2), crd(:, 3), r(1), r(2), r(3));
        if valid
            % Source points supposed to be on the surface. This means, that
            % one of solutions will be below the surface.
            if (norm(d1) < norm(d2))
                tgt_ecef = [ tgt_ecef, d1 ];
            else
                tgt_ecef = [ tgt_ecef, d2 ];
            end
        end
    end
end
