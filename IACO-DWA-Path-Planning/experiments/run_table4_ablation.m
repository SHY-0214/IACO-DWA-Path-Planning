addpath(fileparts(fileparts(mfilename('fullpath'))));
[root,sim,aco,dwa,base]=setup_project();
save_settings(root,'table4',sim,aco,dwa,base);
world=load_world(root,sim.dynamic_map); graph=grid_graph(world,sim.start,sim.goal);
variants=[{'configured'},fieldnames(base)'];
records=cell(0,11);
for trial=1:sim.trials
    seed=sim.seed+trial-1;
    for index=1:numel(variants)
        flags=base; name=variants{index};
        if index>1, flags.(name)=false; end
        rng(seed,'twister'); plan=IACO_main(world,graph,aco,flags,dwa.radius);
        if isempty(plan.path)
            row={name,seed,false,false,false,NaN,NaN,NaN,NaN,NaN,NaN};
        else
            out=simulate_robot(world,plan.path,sim,dwa,aco,flags);
            length_m=out.length;
            if ~out.success, length_m=NaN; end
            row={name,seed,true,out.success,out.collision,length_m,plan.convergence,...
                count_turns(plan.anchors,dwa.turn_threshold),out.min_gap,out.reconnect,out.angular_rms};
        end
        records(end+1,:)=row; %#ok<SAGROW>
    end
    raw=cell2table(records,'VariableNames',{'setting','seed','planned','success','collision','length_m','iterations','turns','min_gap_m','reconnect_s','omega_rms'});
    writetable(raw,fullfile(root,'results','logs','table4_trials.csv'));
end
rows=cell(0,11);
for index=1:numel(variants)
    data=raw(strcmp(raw.setting,variants{index}),:);
    rows(end+1,:)={variants{index},height(data),100*mean(data.success),100*mean(data.collision),...
        mean(data.length_m,'omitnan'),mean(data.iterations,'omitnan'),mean(data.turns,'omitnan'),...
        mean(data.min_gap_m,'omitnan'),mean(data.reconnect_s(data.success),'omitnan'),...
        mean(data.omega_rms,'omitnan'),sum(data.success & isfinite(data.reconnect_s))}; %#ok<SAGROW>
end
summary=cell2table(rows,'VariableNames',{'setting','trials','success_pct','collision_pct','length_m','iterations',...
    'turns','min_gap_m','reconnect_s','omega_rms','reconnect_samples'});
writetable(summary,fullfile(root,'results','logs','table4_computed.csv'));
