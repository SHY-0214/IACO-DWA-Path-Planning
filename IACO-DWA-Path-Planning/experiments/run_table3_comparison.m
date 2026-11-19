addpath(fileparts(fileparts(mfilename('fullpath'))));
[root,sim,aco,dwa,flags]=setup_project();
save_settings(root,'table3',sim,aco,dwa,flags);
records=cell(0,8);
for scene=1:numel(sim.map_files)
    world=load_world(root,sim.map_files{scene}); graph=grid_graph(world,sim.starts(scene,:),sim.goals(scene,:));
    for trial=1:sim.trials
        seed=sim.seed+trial-1;
        for method=["ACO","A*","IACO"]
            rng(seed,'twister'); timer=tic;
            switch method
                case "ACO"
                    r=ACO_classic(graph,aco); anchors=r.path;
                case "A*"
                    r.path=graph.xy(A_star_init(graph),:); r.convergence=NaN; anchors=r.path;
                otherwise
                    r=IACO_main(world,graph,aco,flags,dwa.radius);
                    anchors=[]; if ~isempty(r.path), anchors=r.anchors; end
            end
            elapsed=toc(timer); ok=~isempty(r.path); length_m=NaN; turns=NaN;
            if ok, length_m=calc_path_length(r.path); turns=count_turns(anchors,dwa.turn_threshold); end
            records(end+1,:)={sim.map_sizes(scene),method,seed,ok,length_m,r.convergence,turns,elapsed}; %#ok<SAGROW>
        end
    end
end
raw=cell2table(records,'VariableNames',{'map','method','seed','success','length_m','iterations','turns','runtime_s'});
writetable(raw,fullfile(root,'results','logs','table3_trials.csv'));
rows=cell(0,8);
for n=sim.map_sizes
    for name=["ACO","A*","IACO"]
        data=raw(raw.map==n & string(raw.method)==name,:);
        rows(end+1,:)={n,name,height(data),100*mean(data.success),mean(data.length_m,'omitnan'),...
            mean(data.iterations,'omitnan'),mean(data.turns,'omitnan'),mean(data.runtime_s)}; %#ok<SAGROW>
    end
end
summary=cell2table(rows,'VariableNames',{'map','method','trials','success_pct','length_m','iterations','turns','runtime_s'});
writetable(summary,fullfile(root,'results','logs','table3_computed.csv'));
