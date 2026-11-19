function result=colony_search(graph,cfg,flags)
% Vectorize roulette selection across active ants; each ant has a tabu set.
neighbors=graph.next; count=size(neighbors,1); ants=cfg.pop_size;
prior=ones(count,1)*cfg.tau0;
if flags.use_A_star_prior
    route=A_star_init(graph);
    distance=inf(count,1);
    for index=2:numel(route)
        origin=graph.xy(route(index-1),:); edge=graph.xy(route(index),:)-origin;
        fraction=max(0,min(1,(graph.xy-origin)*edge'/sum(edge.^2)));
        distance=min(distance,vecnorm(graph.xy-origin-fraction.*edge,2,2));
    end
    prior=cfg.tau0./(1+cfg.spatial_decay*distance);
    prior(route)=cfg.prior_gain*cfg.tau0;
end
safe_next=max(1,neighbors);
pheromone=reshape(prior(safe_next),size(neighbors));
heuristic=ones(count,1)./graph.step;
turn=ones(9,8);
if flags.use_potential_field
    heuristic=heuristic.*exp(-cfg.goal_decay*reshape(graph.to_goal(safe_next),size(neighbors)));
    delta=graph.angle-graph.angle';
    turn(1:8,:)=1./(1+cfg.turn_cost*abs(atan2(sin(delta),cos(delta))));
end
result.history=nan(cfg.max_iter,1); result.route=[]; result.length=inf;
for iter=1:cfg.max_iter
    visited=false(count,ants); visited(graph.start,:)=true;
    trail=zeros(ants,count); directions=zeros(ants,count);
    trail(:,1)=graph.start; last=9*ones(ants,1); costs=zeros(ants,1);
    active=(1:ants)'; finished=zeros(ants,1);
    attractiveness=pheromone.^cfg.alpha.*heuristic.^cfg.beta;
    for step=1:count-1
        current=trail(active,step);
        choices=neighbors(current,:);
        blocked=visited(max(1,choices)+(active-1)*count);
        weights=attractiveness(current,:).*turn(last(active),:).^cfg.beta;
        weights(choices==0 | blocked)=0;
        total=sum(weights,2); valid=total>0;
        active=active(valid); weights=weights(valid,:); total=total(valid);
        if isempty(active), break; end
        pick=1+sum(cumsum(weights,2)<=rand(numel(active),1).*total,2);
        pick=min(8,pick);
        node=neighbors(sub2ind(size(neighbors),trail(active,step),pick));
        visited(node+(active-1)*count)=true;
        trail(active,step+1)=node; directions(active,step)=pick;
        costs(active)=costs(active)+reshape(graph.step(pick),[],1);
        last(active)=pick; done=node==graph.goal;
        finished(active(done))=step+1; active=active(~done);
        if isempty(active), break; end
    end
    rho=cfg.fixed_rho;
    if flags.use_adaptive_evaporation
        rho=cfg.rho_min+(cfg.rho_max-cfg.rho_min)*exp(-cfg.phi*((iter-1)/cfg.max_iter)^2);
    end
    pheromone=(1-rho)*pheromone;
    winners=find(finished>0);
    if isempty(winners), pheromone=max(pheromone,cfg.tau_floor); continue; end
    lengths=costs(winners); [shortest,index]=min(lengths);
    result.history(iter)=shortest;
    if shortest<result.length
        ant=winners(index); result.length=shortest; result.route=trail(ant,1:finished(ant));
    end
    average=mean(lengths); worst=max(lengths);
    for ant=winners'
        length_m=costs(ant); gain=1;
        if flags.use_adaptive_evaporation
            if length_m<=average
                gain=1+cfg.reward*(average-length_m)/max(eps,average-shortest);
            else
                gain=-cfg.penalty*(length_m-average)/max(eps,worst-average);
            end
        end
        steps=1:finished(ant)-1;
        edges=sub2ind(size(neighbors),trail(ant,steps),directions(ant,steps));
        pheromone(edges)=pheromone(edges)+cfg.Q*gain/length_m;
    end
    pheromone=max(cfg.tau_floor,pheromone);
end
result.convergence=find(result.history==result.length,1);
if isempty(result.convergence), result.convergence=NaN; end
end
