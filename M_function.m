function M_out = M_function(q)
    m1 = 1; m2 = 1;
    l1 = 1; l2 = 1;
    I1 = 0.1; I2 = 0.1;  
    
    M11 = m1*l1^2/3 + m2*(l1^2 + l2^2/3 + l1*l2*cos(q(2))) + I1 + I2;
    M12 = m2*(l2^2/3 + l1*l2/2*cos(q(2))) + I2;
    M21 = M12;
    M22 = m2*l2^2/3 + I2;
    
    M_out = [M11, M12; M21, M22];
end
