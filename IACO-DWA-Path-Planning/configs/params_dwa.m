function cfg = params_dwa()
cfg.vmax = 0.5;
cfg.wmax = 1.2;
cfg.accel = 0.2;
cfg.angular_accel = 0.5;
cfg.dt = 0.2;
cfg.horizon = 1.8;
cfg.v_samples = 6;
cfg.w_samples = 15;
cfg.weights = [2.6 0.15 0.6 0.55]; % heading, clearance, speed, dynamic risk
cfg.radius = 0.17;
cfg.margin = 0.04;
cfg.lookahead = 1.1;
cfg.tolerance = 0.14;
cfg.warning_range = 3;
cfg.clearance_scale = 1;
cfg.ttc_threshold=2; cfg.trigger_score=0.6; cfg.failure_score=-1;
cfg.clear_frames=4; cfg.lateral_limit=0.55; cfg.heading_limit=pi/4;
cfg.replan_limit=2; cfg.stuck_time=3; cfg.recovery_speed=0.12;
cfg.turn_threshold=pi/12; cfg.reconnect_blend=1.5;
cfg.recovery_durations=[0.6 1.4 1.6 2.5];
end
