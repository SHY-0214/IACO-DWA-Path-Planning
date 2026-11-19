function graph=grid_graph(world,start_xy,goal_xy)
[rows,cols]=size(world.grid);
[y,x]=ndgrid(1:rows,1:cols);
graph.xy=([x(:),y(:)]-0.5)*world.cell;
index=@(point) sub2ind([rows cols],floor(point(2)/world.cell)+1,floor(point(1)/world.cell)+1);
graph.start=index(start_xy); graph.goal=index(goal_xy);
assert(~world.grid(graph.start) && ~world.grid(graph.goal),'Occupied endpoint');
moves=[1 0;1 1;0 1;-1 1;-1 0;-1 -1;0 -1;1 -1];
graph.next=zeros(numel(x),8);
for node=find(~world.grid(:))'
    for direction=1:8
        dx=moves(direction,1); dy=moves(direction,2);
        col=x(node)+dx; row=y(node)+dy;
        if row<1 || row>rows || col<1 || col>cols || world.grid(row,col), continue; end
        if dx && dy && (world.grid(y(node),col)||world.grid(row,x(node))), continue; end
        graph.next(node,direction)=sub2ind([rows cols],row,col);
    end
end
graph.step=vecnorm(moves,2,2)'*world.cell;
graph.angle=atan2(moves(:,2),moves(:,1))';
graph.to_goal=vecnorm(graph.xy-graph.xy(graph.goal,:),2,2);
end
