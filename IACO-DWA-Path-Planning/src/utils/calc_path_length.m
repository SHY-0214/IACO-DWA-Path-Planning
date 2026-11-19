function length_m = calc_path_length(path)
length_m = sum(vecnorm(diff(path,1,1),2,2));
end
