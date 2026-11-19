function out=simulate_robot(world,initial,sim,cfg,aco,flags)
% Closed-loop differential drive, ideal range-limited obstacle-state sensing.
state=[sim.start sim.heading 0 0]; path=densify(initial,cfg.lookahead/4);
count=ceil(sim.timeout/cfg.dt);
log=nan(count,12); cursor=1; avoid=false; clear_count=0; avoiding_since=NaN;
delays=[]; previous_exit=-inf; stage=0; stage_start=0; stuck_frames=0; recovery_count=0;
wrap=@(angle) atan2(sin(angle),cos(angle));
for step=1:count
    time=(step-1)*cfg.dt; xy=state(1:2);
    objects=objects_at(time,sim);
    sensed=objects(vecnorm(objects(:,1:2)-xy,2,2)<=cfg.warning_range+cfg.vmax*cfg.horizon+1,:);
    [deviation,offset]=min(vecnorm(path(cursor:end,:)-xy,2,2));
    nearest=cursor+offset-1;
    visible=false; candidate=nearest;
    if flags.use_reconnection
        for j=nearest:size(path,1)
            if norm(path(j,:)-xy)>2*cfg.lookahead && j>nearest, break; end
            if ~check_collision(xy,path(j,:),world,cfg.radius+cfg.margin)
                visible=true; candidate=j; break
            end
        end
    else
        visible=true;
    end
    cursor=max(cursor,candidate);
    target=path(cursor,:);
    for j=cursor+1:size(path,1)
        if norm(path(j,:)-xy)>cfg.lookahead || check_collision(xy,path(j,:),world,cfg.radius+cfg.margin), break; end
        target=path(j,:);
    end
    speed_limit=cfg.vmax; preferred=[]; heading_error=wrap(atan2(target(2)-xy(2),target(1)-xy(1))-state(3));
    if flags.use_reconnection && (avoid || time-previous_exit<cfg.reconnect_blend) && ...
            (deviation>cfg.lateral_limit || abs(heading_error)>cfg.heading_limit)
        speed_limit=cfg.vmax/2;
    end
    if flags.use_failure_recovery && stage>0
        elapsed=time-stage_start; speed_limit=cfg.recovery_speed;
        if stage==1, preferred=[0 0];
        elseif stage==2, preferred=[0 cfg.wmax/2];
        elseif stage==3, preferred=[-cfg.recovery_speed 0];
        elseif stage==4
            target=xy+cfg.lookahead*[cos(state(3)+pi/3),sin(state(3)+pi/3)];
        end
        if elapsed>=cfg.recovery_durations(stage)
            stage=stage+1; stage_start=time;
        end
    end
    if stage==5 || (flags.use_reconnection && deviation>cfg.replan_limit && time-previous_exit>cfg.reconnect_blend)
        % Join a visible downstream subgoal through a fresh IACO subpath.
        join=min(size(path,1),cursor+ceil(4*cfg.lookahead/(cfg.lookahead/4)));
        local=world;
        for k=1:size(sensed,1)
            [yy,xx]=ndgrid(1:size(local.grid,1),1:size(local.grid,2));
            occupied=hypot((xx-.5)*local.cell-sensed(k,1),(yy-.5)*local.cell-sensed(k,2))<sensed(k,5)+cfg.radius+local.cell/2;
            local.grid=local.grid|occupied;
        end
        [ry,cx]=find(local.grid); local.boxes=[cx-1,ry-1,cx,ry]*local.cell;
        cellfrom=floor(xy/local.cell)+1; cellto=floor(path(join,:)/local.cell)+1;
        if ~local.grid(cellfrom(2),cellfrom(1)) && ~local.grid(cellto(2),cellto(1)) && any(cellfrom~=cellto)
            graph=grid_graph(local,xy,path(join,:)); result=IACO_main(local,graph,aco,flags,cfg.radius);
            if ~isempty(result.path) && ~check_collision(xy,result.path(1,:),local,cfg.radius) && ~check_collision(result.path(end,:),path(join,:),local,cfg.radius)
                path=densify([xy;result.path;path(join:end,:)],cfg.lookahead/4); cursor=1; target=path(min(2,end),:);
            end
        end
        previous_exit=time; stage=0; stuck_frames=0;
    end
    if norm(xy-sim.goal)<=cfg.tolerance, preferred=[0 0]; speed_limit=0; end
    priority=max(double(avoid),max(0,1-(time-previous_exit)/cfg.reconnect_blend));
    [cmd,info]=DWA_core(state,target,world,sensed,cfg,flags,speed_limit,priority,preferred);
    if info.alert && ~avoid
        avoid=true; avoiding_since=time;
        [cmd,info]=DWA_core(state,target,world,sensed,cfg,flags,speed_limit,1,preferred);
    end
    if info.alert, clear_count=0; else, clear_count=clear_count+1; end
    if avoid && ~info.alert && clear_count>=cfg.clear_frames && visible
        avoid=false; previous_exit=time;
    end
    if ~avoid && isfinite(avoiding_since) && deviation<cfg.lateral_limit && abs(heading_error)<cfg.heading_limit
        % Delay from clearance to geometric reacquisition, including clear frames.
        delays(end+1)=time-previous_exit+cfg.clear_frames*cfg.dt; %#ok<AGROW>
        avoiding_since=NaN;
    end
    if abs(cmd(1))<0.015, stuck_frames=stuck_frames+1; else, stuck_frames=0; end
    window=max(2,round(cfg.stuck_time/cfg.dt));
    repeated=step>2*window && norm(xy-log(step-2*window,2:3))<0.08 && sum(abs(log(step-2*window:step-1,5)))*cfg.dt>0.3;
    if flags.use_failure_recovery && stage==0 && norm(xy-sim.goal)>cfg.tolerance && (info.failed || stuck_frames*cfg.dt>=cfg.stuck_time || repeated)
        stage=1; stage_start=time; recovery_count=recovery_count+1;
    end
    half=cmd(2)*cfg.dt/2; factor=1; if abs(half)>eps, factor=sin(half)/half; end
    next=xy+cmd(1)*cfg.dt*factor*[cos(state(3)+half) sin(state(3)+half)];
    state=[next wrap(state(3)+2*half) cmd];
    next_objects=objects_at(time+cfg.dt,sim);
    gap=min(vecnorm(next_objects(:,1:2)-next,2,2)-next_objects(:,5)-cfg.radius);
    if ~isempty(world.boxes)
        displacement=max(0,max(world.boxes(:,1:2)-next,next-world.boxes(:,3:4)));
        gap=min(gap,min(vecnorm(displacement,2,2))-cfg.radius);
    end
    gap=min([gap,next-cfg.radius,world.extent-next-cfg.radius]);
    collided=gap<=0 || check_collision(xy,next,world,0);
    log(step,:)=[time+cfg.dt state gap avoid stage deviation next_objects(1,1:2)];
    arrived=norm(next-sim.goal)<=cfg.tolerance && all(abs(cmd)<1e-6);
    if arrived || collided, break; end
end
out.log=log(1:step,:); out.success=arrived && ~collided; out.collision=collided;
out.min_gap=min(out.log(:,7)); out.length=calc_path_length([sim.start;out.log(:,2:3)]);
out.angular_rms=sqrt(mean(out.log(:,6).^2)); out.recovery_count=recovery_count;
out.reconnect=mean(delays); % NaN if no completed event
end

function dense=densify(path,step)
dense=path(1,:);
for k=2:size(path,1)
    delta=path(k,:)-path(k-1,:); n=ceil(norm(delta)/step);
    if n>0, dense=[dense;path(k-1,:)+(1:n)'/n.*delta]; end %#ok<AGROW>
end
end

function objects=objects_at(time,sim)
route=sim.moving_route; distance=time*sim.moving_speed; pos=route(end,:); vel=[0 0];
for k=2:size(route,1)
    delta=route(k,:)-route(k-1,:); length_m=norm(delta);
    if distance<length_m
        pos=route(k-1,:)+distance/length_m*delta; vel=sim.moving_speed*delta/length_m; break
    end
    distance=distance-length_m;
end
objects=[pos vel sim.moving_radius;sim.fixed_disc(1:2) 0 0 sim.fixed_disc(3)];
end
