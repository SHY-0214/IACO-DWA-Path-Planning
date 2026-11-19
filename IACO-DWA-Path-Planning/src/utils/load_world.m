function world=load_world(root,file)
data=load(fullfile(root,'maps',file));
world.grid=logical(data.occupancy);
world.cell=data.cell_m;
[rows,cols]=find(world.grid);
world.boxes=[cols-1,rows-1,cols,rows]*world.cell;
world.extent=fliplr(size(world.grid))*world.cell;
end
