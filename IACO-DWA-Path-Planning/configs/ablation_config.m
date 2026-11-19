function flags = ablation_config()
flags.use_A_star_prior = true;
flags.use_potential_field = true;
flags.use_adaptive_evaporation = true;
flags.use_triangular_pruning = true;
flags.use_bspline = true;
flags.use_dynamic_risk = true;
flags.use_reconnection = true;
flags.use_failure_recovery = true;
end
