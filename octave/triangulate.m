%
% Triangulation
%
% Input: Path -- parray of oints
% Output: Tv -- triangle vertexes (3 indexes of points from Path)
%         Te -- triangle edges (lengths pf each vtiangle edge)
function [ tv, te ] = triangulate(path);
    nv = size(path, 2);         % number of vertexes

    [ ne, el ] = edges(nv, path);
    [ nt, tv, te ] = trigen(nv, ne, el);
end

function [ ne, el ] = edges(nv, vert)
    ne = nv * (nv - 1) / 2;     % number of edges
    el = zeros(ne, 1);          % edges lengths

    k = 1;
    for i = 1:nv-1
        for j = i+1:nv
            el(k) = norm(vert(:, i) - vert(:, j));
            k = k + 1;
        end
    end
end

function [ nt, tv, te ] = trigen(nv, ne, el)
    nt = nv * (nv - 1) * (nv - 2) / 6;  % maximal number of triangles
    tv = [];                            % triangle vertexes (indexes)
    te = [];                            % triangle edges (length)

    thr = median(el) * 0.75;
    el2 = el .^ 2;

    an = 0;
    a = 0;

    for i = 1:ne
        while i > an
            a = a + 1;
            an = idx_an(nv, a);
        end

        if el(i) < thr
            continue;
        end

        b = nv - an + i;

        for j = i+1:an
            if el(j) < thr
                continue;
            end

            c = nv - an + j;
            k = idx_ab(nv, b, c);

            if el(k) < thr
                continue;
            end

            if (el2(i) - el2(j) - el2(k) > 0 ||
                el2(k) - el2(i) - el2(j) > 0 ||
                el2(j) - el2(k) - el2(i) > 0)
                continue;
            end

            tv = [ tv; a, b, c ];
            te = [ te; el([ i, j, k ]) ];
        end
    end
end

function i = idx_an(n, a)
    i = a / 2 * (2 * n - a - 1);
end

function i = idx_ab(n, a, b)
    i = idx_an(n, a) - (n - b);
end
