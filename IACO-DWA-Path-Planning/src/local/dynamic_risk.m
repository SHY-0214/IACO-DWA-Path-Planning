function [risk,min_gap,ttc]=dynamic_risk(px,py,vx,vy,times,discs,radius)
% discs: x,y,vx,vy,r; prediction is constant velocity over each window.
risk=zeros(1,size(px,2)); min_gap=inf(size(risk)); ttc=inf(size(risk));
for index=1:size(discs,1)
    obj=discs(index,:);
    dx=obj(1)+times*obj(3)-px; dy=obj(2)+times*obj(4)-py;
    distance=hypot(dx,dy); gap=distance-radius-obj(5);
    closing=(dx.*(vx-obj(3))+dy.*(vy-obj(4)))./max(eps,distance);
    risk=max(risk,max((0.1+max(0,closing))./(0.2+max(0,gap)),[],1));
    min_gap=min(min_gap,min(gap,[],1));
    collision_time=gap./max(closing,eps); collision_time(closing<=0)=inf;
    ttc=min(ttc,min(collision_time,[],1));
end
end
