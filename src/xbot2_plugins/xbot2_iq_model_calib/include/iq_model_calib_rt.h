#ifndef IQ_MODEL_CALIB_RT
#define IQ_MODEL_CALIB_RT

#include <matlogger2/matlogger2.h>

#include <xbot2/xbot2.h>

#include <cartesian_interface/CartesianInterfaceImpl.h>

#include <thread>

#include "calib_utils.hpp"

#include <std_msgs/Bool.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <xbot2/hal/dev_ft.h>

#include <xbot2/ros/ros_support.h>
#include <cartesian_interface/ros/RosServerClass.h>

#include <cartesian_interface/sdk/rt/LockfreeBufferImpl.h>

#include <awesome_leg/IqEstStatus.h>

using namespace XBot;
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
        _verbose = false,
        _jnt_names_were_set = false;

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

    Eigen::VectorXd _q_p_meas, _q_p_dot_meas, _q_p_ddot_est, _tau_meas,
                    _K_t, _K_d0, _K_d1, _rot_MoI, _red_ratio;

    std::vector<float> _iq_est;
    std::vector<std::string> _iq_jnt_names;

    MatLogger2::Ptr _dump_logger;

    // handle adapting ROS primitives for RT support
    RosSupport::UniquePtr _ros;

    // queue object to handle multiple subscribers/servers at once
    CallbackQueue _queue;

    SubscriberPtr<xbot_msgs::CustomState> _aux_signals_sub;
    SubscriberPtr<xbot_msgs::JointState> _js_signals_sub;

    PublisherPtr<awesome_leg::IqEstStatus> _iq_est_pub;

    IqRosGetter _iq_getter;
    IqEstimator _iq_estimator;
    NumDiff _num_diff;

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

    void pub_iq_est();
                 
};

#endif // IQ_MODEL_CALIB_RT
