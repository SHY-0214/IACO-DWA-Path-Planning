function global_figures(number)
[root,sim,aco,dwa,flags]=setup_project();
save_settings(root,sprintf('fig%d',number),sim,aco,dwa,flags);
fig=figure('Color','w','Position',[100 100 1350 430]); tiledlayout(1,3);
for scene=1:numel(sim.map_files)
    world=load_world(root,sim.map_files{scene});
    graph=grid_graph(world,sim.starts(scene,:),sim.goals(scene,:));
    rng(sim.seed,'twister'); classic=ACO_classic(graph,aco);
    rng(sim.seed,'twister'); improved=IACO_main(world,graph,aco,flags,dwa.radius);
    n=sim.map_sizes(scene); nexttile; hold on;
    if number==6
        plot_functions(world);
        prior=graph.xy(A_star_init(graph),:);
        reference=readmatrix(fullfile(root,'reference',sprintf('ref_path_%d.csv',n)));
        paths={classic.path,prior,reference,improved.path};
        labels={'ACO','A*','Ref-ACO (digitized)','IACO'};
        colors=lines(4);
        for k=1:4
            path=paths{k}; if isempty(path), continue; end
            plot(path(:,1),path(:,2),'Color',colors(k,:),'LineWidth',1.5,'DisplayName',labels{k});
            writematrix(path,fullfile(root,'results','logs',sprintf('fig6_%d_method%d.csv',n,k)));
        end
    else
        ref=readmatrix(fullfile(root,'reference',sprintf('ref_curve_%d.csv',n)));
        plot(classic.history,'DisplayName','ACO'); plot(ref(:,1),ref(:,2),'DisplayName','Ref-ACO (digitized)');
        plot(improved.history,'DisplayName','IACO'); xlabel('Iteration'); ylabel('Generation best grid length (m)');
        writetable(table((1:aco.max_iter)',classic.history,improved.history,'VariableNames',{'iteration','aco_m','iaco_m'}),...
            fullfile(root,'results','logs',sprintf('fig7_%d.csv',n)));
    end
    title(sprintf('%d × %d',n,n)); legend('show','Location','best');
end
save_figure(fig,root,sprintf('fig%d',number));
end
