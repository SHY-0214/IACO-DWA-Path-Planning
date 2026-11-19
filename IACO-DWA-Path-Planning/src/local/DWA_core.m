function [cmd,info]=DWA_core(state,target,world,discs,cfg,flags,speed_limit,priority,preferred)
% preferred=[v,w] during a recovery manoeuvre; otherwise use heading feedback.
wrap=@(a) atan2(sin(a),cos(a));
dist=norm(target-state(1:2)); yaw=state(3);
error=wrap(atan2(target(2)-state(2),target(1)-state(1))-yaw);
if isempty(preferred)
    preferred=[min(speed_limit,sqrt(2*cfg.accel*dist))*max(0,cos(error)),2*error];
end
low=max([min(0,preferred(1)),-cfg.wmax],state(4:5)-cfg.dt*[cfg.accel,cfg.angular_accel]);
low(1)=max(-cfg.recovery_speed,min(low(1),state(4)+cfg.accel*cfg.dt));
high=min([cfg.vmax,cfg.wmax],state(4:5)+cfg.dt*[cfg.accel,cfg.angular_accel]);
high(1)=max(low(1),high(1));
nominal=min(high,max(low,preferred));
[v,w]=meshgrid(linspace(low(1),high(1),cfg.v_samples),linspace(low(2),high(2),cfg.w_samples));
v=[v(:)' nominal(1)]; w=[w(:)' nominal(2)];
times=(cfg.dt:cfg.dt:cfg.horizon)'; half=times*w/2;
scale=ones(size(half)); nonzero=abs(half)>eps; scale(nonzero)=sin(half(nonzero))./half(nonzero);
x=state(1)+(times*v).*scale.*cos(yaw+half); y=state(2)+(times*v).*scale.*sin(yaw+half);
clearance=min(min(x,world.extent(1)-x),min(y,world.extent(2)-y));
reach=cfg.vmax*cfg.horizon+cfg.clearance_scale+cfg.radius;
boxes=world.boxes(all(world.boxes(:,1:2)<state(1:2)+reach & world.boxes(:,3:4)>state(1:2)-reach,2),:);
for index=1:size(boxes,1)
    box=boxes(index,:);
    clearance=min(clearance,hypot(max(0,max(box(1)-x,x-box(3))),max(0,max(box(2)-y,y-box(4)))));
end
static=min(clearance,[],1)-cfg.radius;
[risk,gap,ttc]=dynamic_risk(x,y,v.*cos(yaw+2*half),v.*sin(yaw+2*half),times,discs,cfg.radius);
brake=v.^2/(2*cfg.accel)+cfg.margin+cfg.dt*abs(v)/2;
valid=min(gap,static)>brake;
heading=1-abs(wrap(atan2(target(2)-y(end,:),target(1)-x(end,:))-yaw-times(end)*w))/pi;
weights=cfg.weights;
score=weights(1)*heading+weights(2)*min(1,static/cfg.clearance_scale)+weights(3)*(1-abs(v-preferred(1))/cfg.vmax);
trigger_score=score(end);
if flags.use_dynamic_risk
    trigger_score=trigger_score-weights(4)*risk(end);
    score=score-priority*weights(4)*risk;
end
info.alert=any(vecnorm(discs(:,1:2)-state(1:2),2,2)-discs(:,5)<cfg.warning_range) || ttc(end)<cfg.ttc_threshold || trigger_score<cfg.trigger_score;
score(~valid)=-inf; [info.score,index]=max(score);
info.failed=~isfinite(info.score)||info.score<cfg.failure_score;
if info.failed
    cmd=sign(state(4:5)).*max(0,abs(state(4:5))-cfg.dt*[cfg.accel cfg.angular_accel]);
elseif valid(end) && (~info.alert && priority==0 || preferred(1)<0 || speed_limit<=cfg.recovery_speed)
    cmd=nominal;
else
    cmd=[v(index),w(index)];
end
end
