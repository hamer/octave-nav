%
% Trilateration
%
% Input: A, B, C -- points; rA, rB, rC -- distances to target
% Output: D1, D2 -- solutions
%
% Method: http://en.wikipedia.org/wiki/Trilateration
%   change coordinate system, points A, B, C should have following coordinates
%       A'(0, 0, 0), B'(d, 0, 0), C'(i, j, 0)
%   basis is (e_x, e_y, e_z), origin is A
%
%   find points D1'(x, y, z), D2'(x, y, -z) of intersections of three spheres
%       (A', rA), (B', rB), (C', rC)
%
%   convert points D1', D2' to source coordinate system
function [ D1, D2, valid ] = trilat(A, B, C, rA, rB, rC)
    X  = B - A;
    AC = C - A;

    r2 = [ rA, rB, rC ] .^ 2;

    d = norm(X);
    e_x = X / d;

    i = dot(e_x, AC);

    Y = AC - i * e_x;
    e_y = Y / norm(Y);

    j = dot(e_y, AC);

    e_z = cross(e_x, e_y);

    x = (r2(1) - r2(2) + d^2) / (2 * d);
    y = (r2(1) - r2(3) + i^2 + j^2) / (2 * j) - i * x / j;
    z = sqrt(r2(1) - x^2 - y^2);

    D1 = A + x * e_x + y * e_y + z * e_z;
    D2 = A + x * e_x + y * e_y - z * e_z;
    valid = (r2(1) - x^2 - y^2) >= 0;
end
