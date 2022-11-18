#ifndef IQ_MODEL_CALIB
#define IQ_MODEL_CALIB

#include <matlogger2/matlogger2.h>

#include <xbot2/xbot2.h>

#include <cartesian_interface/CartesianInterfaceImpl.h>

#include <thread>

#include "utils.hpp"

#include <std_msgs/Bool.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <xbot2/hal/dev_ft.h>

#include <xbot2/ros/ros_support.h>
#include <cartesian_interface/ros/RosServerClass.h>

#include <cartesian_interface/sdk/rt/LockfreeBufferImpl.h>


using namespace XBot;
using namespace XBot::Cartesian;
using namespace CalibUtils;
using namespace SignProcUtils;

//typedef Eigen::Array<bool, Eigen::Dynamic, 1> VectorXb;

/**
 * @brief The IqModelCalibRt class is a ControlPlugin
 * implementing ......
**/

class IqModelCalibRt : public ControlPlugin
{

public:

    using ControlPlugin::ControlPlugin;

    // initialization method; the plugin won't be run
    // if this returns 'false'
    bool on_initialize() override;

    // callback for the 'Starting' state
    // start_completed() must be called to switch
    // to 'Run' state
    void starting() override;

    // callback for 'Run' state
    void run() override;

    // callback for 'On Stop' state
    void stopping() override;

    // callback for 'On Stop' state
    void on_stop() override;

    // callback for 'On Abort' state
    void on_close() override;

    // callback for 'On Abort' state
    void on_abort() override;


private:

    bool _is_first_run = true,
        _pause_started = false, _pause_finished = false,
        _rt_active, _nrt_exit,
        _is_sim = true,
        _reduce_dumped_sol_size = false,
        _verbose = false;

    int _n_jnts_model,
        _n_jnts_model_ft_est,
        _n_jnts_robot,
        _sample_index = 0;

    std::string _mat_path, _mat_name, _dump_mat_suffix,
                _urdf_path, _srdf_path,
                _hw_type;

    double _plugin_dt,
        _loop_time = 0.0, _loop_timer_reset_time = 3600.0,
        _matlogger_buffer_size = 1e4;

    Eigen::VectorXd _q_p_meas, _q_p_dot_meas, _tau_meas;

    MatLogger2::Ptr _dump_logger;

    // handle adapting ROS primitives for RT support
    RosSupport::UniquePtr _ros;

    // queue object to handle multiple subscribers/servers at once
    CallbackQueue _queue;

    SubscriberPtr<xbot_msgs::CustomState> _aux_signals_sub;

    AuxSigDecoder _aux_sig_decoder;

    void get_params_from_config();

    void init_vars();
    void init_clocks();
    void init_dump_logger();

    void reset_flags();
    
    void update_state();

    void update_clocks();

    void add_data2dump_logger();
    void add_data2bedumped();

    void spawn_nrt_thread();

    void init_nrt_ros_bridge();

    void is_sim(std::string sim_string);
                 
};

#endif // IQ_MODEL_CALIB
