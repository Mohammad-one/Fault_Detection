    function Vm_out = Vm_function(q, q_dot)
        Vm_out = [-sin(q(2)) * q_dot(2), -sin(q(2)) * (q_dot(1) + q_dot(2));
                   sin(q(2)) * q_dot(1), 0];
    end
    