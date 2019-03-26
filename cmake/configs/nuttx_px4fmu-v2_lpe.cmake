include(configs/nuttx_px4fmu-v2_default)

set(PARAM_DEFAULT_OVERRIDES "{\\\"SYS_MC_EST_GROUP\\\": 1}")

list(REMOVE_ITEM config_module_list
	modules/ekf2
	)

list(APPEND config_module_list

	drivers/irlock

	modules/attitude_estimator_q
	modules/local_position_estimator
	modules/landing_target_estimator
	drivers/distance_sensor/mb12xx
	drivers/distance_sensor/sf0x
	drivers/distance_sensor/sf1xx
	drivers/distance_sensor/srf02
	drivers/distance_sensor/teraranger
	drivers/distance_sensor/tfmini
	drivers/distance_sensor/ulanding
	)
