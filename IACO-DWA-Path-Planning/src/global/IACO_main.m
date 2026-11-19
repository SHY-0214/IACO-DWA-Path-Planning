function result=IACO_main(world,graph,cfg,flags,radius)
result=colony_search(graph,cfg,flags);
result.path=graph.xy(result.route,:);
if isempty(result.path), return; end
if flags.use_triangular_pruning
    result.path=triangular_pruning(result.path,world,radius);
end
result.anchors=result.path;
if flags.use_bspline
    result.path=bspline_smoothing(result.path,world,radius,cfg.spline_step);
end
end
