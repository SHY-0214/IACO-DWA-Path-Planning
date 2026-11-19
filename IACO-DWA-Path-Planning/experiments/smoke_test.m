% Run from repo root with MATLAB or GNU Octave, no plotting dependencies.
addpath(fileparts(fileparts(mfilename('fullpath'))));
[root,sim,aco,dwa,flags]=setup_project();
for scene=1:numel(sim.map_files)
    world=load_world(root,sim.map_files{scene});
    assert(isequal(size(world.grid),[1 1]*sim.map_sizes(scene)));
    graph=grid_graph(world,sim.starts(scene,:),sim.goals(scene,:));
    assert(~isempty(A_star_init(graph)));
end
world=load_world(root,sim.dynamic_map); graph=grid_graph(world,sim.start,sim.goal);
rng(sim.seed,'twister'); first=IACO_main(world,graph,aco,flags,dwa.radius);
rng(sim.seed,'twister'); again=IACO_main(world,graph,aco,flags,dwa.radius);
assert(~isempty(first.path) && isequal(first.path,again.path) && isequaln(first.history,again.history));
for i=2:size(first.path,1)
    assert(~check_collision(first.path(i-1,:),first.path(i,:),world,dwa.radius));
end
out=simulate_robot(world,first.path,sim,dwa,aco,flags);
fprintf('success=%d collision=%d simulated_s=%.1f travelled_m=%.4f min_gap_m=%.4f\n',...
    out.success,out.collision,out.log(end,1),out.length,out.min_gap);
assert(out.success && ~out.collision,'Default dynamic trial did not arrive');
