function quat = axis2quat(phi, axis)
    cp = cos(phi / 2);
    sp = sin(phi / 2);

    quat = [ cp; sp * axis ];
end
