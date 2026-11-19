function save_figure(fig,root,name)
base=fullfile(root,'results','figures',name);
exportgraphics(fig,[base '.png'],'Resolution',200);
exportgraphics(fig,[base '.pdf'],'ContentType','vector');
end
