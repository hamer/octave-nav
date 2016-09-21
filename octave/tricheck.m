%
% Triangle equilaterality checking
% Input: A, B, C -- points; rA, rB, rC -- distances to target
% Output: Ke -- euilaterality, d -- distances, I -- sphere intersections
%
%   Ke = 1 -- triangle is equilateral
%   Ke > 0 -- triangle is acute
%   Ke = 0 -- triangle is right
%   Ke < 0 -- triangle is obtuse
%
%   d = [ norm(AB), norm(BC), norm(CA) ] -- distances
%
%   I -- intersection of three pairs of spheres
%
function [ Ke, d, I ] = tricheck(A, B, C, rA, rB, rC)
    AB = B - A;
    BC = C - B;
    CA = A - C;

    s = [ dot( AB, -CA), dot(-AB,  BC), dot(-BC,  CA) ];
    Ke = min(s) / max(s);

    d = sqrt([ s(1) + s(2), s(2) + s(3), s(3) + s(1) ]);
    I = [ d(1) - rA < rB && rB < d(1) + rA, ...
          d(2) - rB < rC && rC < d(2) + rB, ...
          d(3) - rC < rA && rA < d(3) + rC ];
end
