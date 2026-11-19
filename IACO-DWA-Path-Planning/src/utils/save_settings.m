function save_settings(root,name,sim,aco,dwa,flags)
meta.version=version; meta.rng_algorithm='twister'; meta.seeds=sim.seed+(0:sim.trials-1);
save(fullfile(root,'results','logs',[name '_settings.mat']),'sim','aco','dwa','flags','meta');
end
