function curve=bspline_smoothing(path,world,radius,step)
% Clamped cubic B-spline via de Boor; no spline toolbox required.
if size(path,1)<2, curve=path; return; end
control=path(1,:);
for index=2:size(path,1)
    edge=path(index,:)-path(index-1,:);
    control=[control;path(index-1,:)+edge/3;path(index,:)-edge/3;path(index,:)]; %#ok<AGROW>
end
count=size(control,1);
knots=[zeros(1,4),(1:(count-4))/(count-3),ones(1,4)];
samples=linspace(0,1,max(2,1+ceil(calc_path_length(path)/step)));
curve=zeros(numel(samples),2);
for index=1:numel(samples)-1
    u=samples(index); span=find(knots<=u,1,'last');
    work=control(span-3:span,:);
    for level=1:3
        for offset=3:-1:level
            left=span-3+offset;
            weight=(u-knots(left))/(knots(left+4-level)-knots(left));
            work(offset+1,:)=(1-weight)*work(offset,:)+weight*work(offset+1,:);
        end
    end
    curve(index,:)=work(4,:);
end
curve(end,:)=path(end,:);
for index=2:size(curve,1)
    if check_collision(curve(index-1,:),curve(index,:),world,radius)
        curve=path; return
    end
end
end
