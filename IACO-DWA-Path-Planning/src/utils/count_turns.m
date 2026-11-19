function count=count_turns(path,threshold)
% Count discrete direction changes larger than the configured tolerance.
delta=diff(path,1,1); delta=delta(vecnorm(delta,2,2)>eps,:);
angle=diff(atan2(delta(:,2),delta(:,1)));
count=sum(abs(atan2(sin(angle),cos(angle)))>threshold);
end
