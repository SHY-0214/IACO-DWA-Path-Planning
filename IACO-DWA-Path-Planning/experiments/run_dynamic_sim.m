addpath(fileparts(fileparts(mfilename('fullpath'))));
[root,sim,aco,dwa,flags]=setup_project();
save_settings(root,'fig8',sim,aco,dwa,flags);
world=load_world(root,sim.dynamic_map);
graph=grid_graph(world,sim.start,sim.goal);
plan=IACO_main(world,graph,aco,flags,dwa.radius);
assert(~isempty(plan.path),'IACO found no path for this seed');
out=simulate_robot(world,plan.path,sim,dwa,aco,flags);
names={'t','x','y','yaw','v','omega','clearance','avoid','recovery','deviation','obs_x','obs_y'};
writetable(array2table(out.log,'VariableNames',names),fullfile(root,'results','logs','fig8_trajectory.csv'));
fig=figure('Color','w'); tiledlayout(2,2);
indices=round(linspace(1,size(out.log,1),4)); circle=linspace(0,2*pi,90);
for frame=indices
    nexttile; plot_functions(world);
    plot(plan.path(:,1),plan.path(:,2),'k--'); plot(sim.moving_route(:,1),sim.moving_route(:,2),'b:');
    plot(out.log(1:frame,2),out.log(1:frame,3),'r','LineWidth',1.6);
    fill(out.log(frame,11)+sim.moving_radius*cos(circle),out.log(frame,12)+sim.moving_radius*sin(circle),'g');
    fill(sim.fixed_disc(1)+sim.fixed_disc(3)*cos(circle),sim.fixed_disc(2)+sim.fixed_disc(3)*sin(circle),'y');
    title(sprintf('t = %.1f s',out.log(frame,1)));
end
save_figure(fig,root,'fig8');
fprintf('Success=%d, collision=%d, minimum clearance=%.3f m\n',out.success,out.collision,out.min_gap);
