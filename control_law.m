%{
function tau = control_law(q, q_dot, q_d, q_dot_d, q_ddot_d, M_fn, Vm_fn, G_fn)
    Kp = 100 * eye(2);
    Kd = 20 * eye(2);

    e = q_d - q;
    e_dot = q_dot_d - q_dot;

    tau = M_fn(q) * (q_ddot_d + Kd * e_dot + Kp * e) + Vm_fn(q, q_dot) * q_dot + G_fn(q);
end
%}
function tau = control_law(q, q_dot, q_d, q_dot_d, q_ddot_d, Kp, Kd, M_fn, Vm_fn, G_fn)
    % Calculate errors
    e = q_d - q;
    e_dot = q_dot_d - q_dot;

    tau = M_fn(q) * (q_ddot_d + Kd * e_dot + Kp * e) + Vm_fn(q, q_dot) * q_dot + G_fn(q);
end
