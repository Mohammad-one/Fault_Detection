function G_out = G_function(q)
    g = 9.81;
    m1 = 1; m2 = 1;
    l1 = 1; l2 = 1;
    G1 = (m1*l1/2 + m2*l1)*g*sin(q(1)) + m2*l2/2*g*sin(q(1) + q(2));
    G2 = m2*l2/2*g*sin(q(1) + q(2));
    G_out = [G1; G2];
end
