% gen_poly(n_edges, l_edge, l_node)
%   n_edges -- number of edges
%   l_edge  -- number of points of movement along the edge
%   l_node  -- number of points of rotation of the node
function [ rs ] = gen_poly(ne, le, ln)
    phi = 2 * pi * (0:ne-1) / ne;
    nodes = [ cos(phi); sin(phi) ];
    edge = ones(1, le);
    node = ones(1, ln);

    if ne < 2
        rs = [ 0; 0; 0 ];
        return;
    end

    rs = [];

    for i = 1:ne
        node_a = nodes(:, 1 + mod(i - 1, ne));
        node_b = nodes(:, 1 + mod(i    , ne));

        dir = pi * (2 * (i-1:i+1) / ne + 0.5);
        yaw_p = dir2rot(dir(1), dir(2));
        yaw_n = dir2rot(dir(2), dir(3));

        if le > 0
            line = [ gen_line(node_a, node_b, le); edge .* yaw_p ];
        else
            line = [];
        end

        if ln > 0
            rot  = [ node .* node_b; gen_rot(yaw_p, yaw_n, ln) ];
        else
            rot = [];
        end

        rs = [ rs, line, rot ];
    end
end

function [ line ] = gen_line(a, b, n)
    line = a + (b - a) .* ((0:n-1) / n);
end

function [ yaw ] = dir2rot(a, b)
    yaw = wrap_2pi((3 * a - b) / 2 + pi / 2);
end

function [ rot ] = gen_rot(a, b, n)
    rot = wrap_2pi(a + ((0:n-1) / n) * wrap_pi(b - a));
end
