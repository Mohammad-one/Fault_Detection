function M_tilde = M_tilde_fn(q, t, fault_time)
    M_nominal = M_function(q);
    if t < fault_time
        M_tilde = M_nominal;
    else
        % Fault injected: increase mass effect in first link by 50%
        M_tilde = M_nominal;
        M_tilde(1,1) = 1.5 * M_nominal(1,1);
    end
end
