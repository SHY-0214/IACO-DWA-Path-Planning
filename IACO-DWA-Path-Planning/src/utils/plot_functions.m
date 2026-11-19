function plot_functions(world)
imagesc(([1:size(world.grid,2)]-.5)*world.cell,([1:size(world.grid,1)]-.5)*world.cell,world.grid);
set(gca,'YDir','normal'); colormap([1 1 1;.19 .22 .25]); clim([0 1]);
axis equal; xlim([0 world.extent(1)]); ylim([0 world.extent(2)]); hold on
xlabel('x (m)'); ylabel('y (m)');
end
