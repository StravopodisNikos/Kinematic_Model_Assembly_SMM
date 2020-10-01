function [passive_under_tform] = add_passive_under_synthetic_joint_tf()
% This is synthetic1 tform defined in tf_list_for_conditioned_assembly.yaml

syn1_rpy = [0 0 0];
syn1_xyz = [0 0 0.048]';

passive_under_tform = eul2tform(syn1_rpy);
passive_under_tform(1:3,4) = syn1_xyz;

end