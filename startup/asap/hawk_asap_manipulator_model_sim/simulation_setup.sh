export USE_IMPEDANCE=true
export ABSOLUTE_CONFIG=true
export MAP_CONFIG=$(rospack find larics_motion_planning)/startup/asap/hawk_asap_manipulator_model_sim/custom_config/hawk_asap_manipulator_model_sim.yaml
export TRAJ_CONFIG=$(rospack find larics_motion_planning)/startup/asap/hawk_asap_manipulator_model_sim/custom_config/hawk_asap_manipulator_model_sim.yaml
export STATE_VALIDITY_CONFIG=$(rospack find larics_motion_planning)/startup/asap/hawk_asap_manipulator_model_sim/custom_config/hawk_asap_manipulator_model_sim.yaml
export KINEMATICS_CONFIG=$(rospack find larics_motion_planning)/startup/asap/hawk_asap_manipulator_model_sim/custom_config/hawk_asap_manipulator_model_sim.yaml
export PATH_PLANNER_CONFIG=$(rospack find larics_motion_planning)/startup/asap/hawk_asap_manipulator_model_sim/custom_config/hawk_asap_manipulator_model_sim.yaml
export MODEL_CORRECTION_CONFIG=$(rospack find larics_motion_planning)/config/model_correction_config_example.yaml
export OCTOMAP_FILE=$(rospack find larics_motion_planning)/config/empty_map.binvox.bt
export DH_PARAMETERS_FILE=$(rospack find aerial_manipulators_control)/config/asap_manipulator_4r_parameters.yaml
