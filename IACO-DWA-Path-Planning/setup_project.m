function [root,sim,aco,dwa,flags]=setup_project()
root=fileparts(mfilename('fullpath'));
addpath(fullfile(root,'configs'),genpath(fullfile(root,'src')),fullfile(root,'experiments'));
sim=params_sim(); aco=params_aco(); dwa=params_dwa(); flags=ablation_config();
rng(sim.seed,'twister');
for name={'logs','figures'}
    directory=fullfile(root,'results',name{1}); if ~isfolder(directory), mkdir(directory); end
end
end
