function route=A_star_init(graph)
count=size(graph.xy,1);
cost=inf(count,1); cost(graph.start)=0;
parent=zeros(count,1); open=false(count,1); open(graph.start)=true;
while any(open)
    candidates=find(open);
    [~,index]=min(cost(candidates)+graph.to_goal(candidates));
    node=candidates(index);
    if node==graph.goal, break; end
    open(node)=false;
    for direction=find(graph.next(node,:)>0)
        next=graph.next(node,direction);
        proposal=cost(node)+graph.step(direction);
        if proposal<cost(next)
            cost(next)=proposal; parent(next)=node; open(next)=true;
        end
    end
end
route=[];
if ~isfinite(cost(graph.goal)), return; end
route=graph.goal;
while route(1)~=graph.start
    route=[parent(route(1)),route]; %#ok<AGROW>
end
end
