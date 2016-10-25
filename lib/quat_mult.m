function m = qut_mult(q_1, q_2)
    m = [ q_1(1) * q_2(1) - q_1(2) * q_2(2) - q_1(3) * q_2(3) - q_1(4) * q_2(4);
          q_1(1) * q_2(2) + q_1(2) * q_2(1) + q_1(3) * q_2(4) - q_1(4) * q_2(3);
          q_1(1) * q_2(3) - q_1(2) * q_2(4) + q_1(3) * q_2(1) - q_1(4) * q_2(2);
          q_1(1) * q_2(4) + q_1(2) * q_2(3) - q_1(3) * q_2(2) + q_1(4) * q_2(1) ];
end