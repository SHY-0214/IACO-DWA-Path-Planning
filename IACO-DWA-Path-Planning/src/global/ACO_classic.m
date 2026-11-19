function result=ACO_classic(graph,cfg)
flags=ablation_config();
flags.use_A_star_prior=false; flags.use_potential_field=false;
flags.use_adaptive_evaporation=false;
result=colony_search(graph,cfg,flags);
result.path=graph.xy(result.route,:);
end
