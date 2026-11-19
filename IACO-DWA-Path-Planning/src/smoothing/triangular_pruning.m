function pruned=triangular_pruning(path,world,radius)
pruned=path(1,:); current=1;
while current<size(path,1)
    next=size(path,1);
    while next>current+1 && check_collision(path(current,:),path(next,:),world,radius)
        next=next-1;
    end
    pruned(end+1,:)=path(next,:); %#ok<AGROW>
    current=next;
end
end
