#!/bin/sh

rosbag record /base_est_rt/base_estimation_node /bt_rt/bt_root/status /bus_power_rt/est_reg_pow_node_iq_model /bus_power_rt/iq_est_node_pow_iq_model /bus_power_rt/iq_meas_node_iq_model /bus_power_rt/meas_reg_pow_node /idler_rt/idle_status /idler_rt/safety_stop_status /impact_detector_rt/impact_state /impact_detector_rt/impact_state /plugins_mngr_rt/plugins_manager/plugins_status /temp_monitor_rt/temp_monitor/temp_status /jmp_replayer_rt/replay_status_node /xbotcore/aux /xbotcore/command /xbotcore/fault /xbotcore/joint_states /xbotcore/statistics
