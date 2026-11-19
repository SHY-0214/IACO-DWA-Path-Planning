function hit = check_collision(start_xy,end_xy,world,radius)
% Continuous segment versus inflated axis-aligned cell boxes (conservative).
lower = min(start_xy,end_xy)-radius;
upper = max(start_xy,end_xy)+radius;
if any(lower<0) || any(upper>world.extent)
    hit=true; return
end
cells = world.boxes;
use = all(cells(:,1:2)<=upper & cells(:,3:4)>=lower,2);
cells = cells(use,:);
entry=zeros(size(cells,1),1); leave=ones(size(entry));
for axis=1:2
    delta=end_xy(axis)-start_xy(axis);
    left=cells(:,axis)-radius-start_xy(axis);
    right=cells(:,axis+2)+radius-start_xy(axis);
    if abs(delta)<eps
        entry(left>0 | right<0)=inf;
    else
        entry=max(entry,min(left/delta,right/delta));
        leave=min(leave,max(left/delta,right/delta));
    end
end
hit=any(entry<=leave);
end
