function psi_val = fault_term(q, q_dot, tau, M_tilde_q, M_fn, Vm_fn, G_fn)
    M_q = M_fn(q);
    Vm_q_qdot = Vm_fn(q, q_dot);
    G_q = G_fn(q);
    
    psi_val = (M_tilde_q \ eye(size(M_tilde_q)) - M_q \ eye(size(M_q))) * (tau - Vm_q_qdot * q_dot - G_q);
end
