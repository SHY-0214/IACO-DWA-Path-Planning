function sim = params_sim()
sim.seed = 42;
sim.trials = 50;
sim.map_sizes = [20 25 30];
sim.map_files = {'map_20x20.mat','map_25x25.mat','map_30x30.mat'};
sim.starts = [0.5 19.5; 0.5 24.5; 0.5 29.5];
sim.goals = [19.5 0.5; 24.5 0.5; 29.5 0.5];
sim.dynamic_map = 'map_dynamic_test.mat';
sim.start = [1.5 29.5];
sim.goal = [29.5 1.5];
sim.heading = -pi/4;
% Approximate the blue moving-object trace in Fig. 8 by linear segments.
sim.moving_route = [4 2.5;9 7.5;16 11;25 15.5];
sim.moving_speed = 0.24;
sim.moving_radius = 0.48;
sim.fixed_disc = [9.5 21.2 0.48]; % x,y,r. Initially unknown to global planner
sim.timeout = 260;
end
