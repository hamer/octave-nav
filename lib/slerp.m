%
% Rotation interpolation
%
% Input: q_1, q_2 -- rotation quaternions
%        h -- coeff (0 -- q_1, 1 -- q_2)
%
function q = slerp(q_1, q_2, h)
    omega = acos(dot(q_1, q_2));

    so = sin(omega);
    if abs(so) < 1e-15
        q = q_1;
        return;
    end

    q = (q_1 * sin(omega * (1 - h)) + q_2 * sin(omega * h)) / so;
end
