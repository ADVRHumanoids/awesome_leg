#include "iq_model_calib.h"

#include <math.h> 

void IqModelCalibRt::init_clocks()
{
    _loop_time = 0.0; // reset loop time clock

    _pause_time = 0.0;
}

void IqModelCalibRt::init_vars()
{

    _q_p_meas = Eigen::VectorXd::Zero(_n_jnts_robot);
    _q_p_dot_meas = Eigen::VectorXd::Zero(_n_jnts_robot);
    _tau_meas = Eigen::VectorXd::Zero(_n_jnts_robot);

}

void IqModelCalibRt::reset_flags()
{

    _sample_index = 0.0; // resetting samples index, in case the plugin stopped and started again


}

void IqModelCalibRt::update_clocks()
{
    // Update timer(s)
    _loop_time += _plugin_dt;

    // Reset timers, if necessary
    if (_loop_time >= _loop_timer_reset_time)
    {
        _loop_time = _loop_time - _loop_timer_reset_time;
    }
    
}

void IqModelCalibRt::get_params_from_config()
{
    // Reading some parameters from XBot2 config. YAML file

    _urdf_path = getParamOrThrow<std::string>("~urdf_path"); 
    _srdf_path = getParamOrThrow<std::string>("~srdf_path"); 

    _mat_path = getParamOrThrow<std::string>("~mat_path"); 
    _mat_name = getParamOrThrow<std::string>("~mat_name"); 
    _dump_mat_suffix = getParamOrThrow<std::string>("~dump_mat_suffix"); 
    _matlogger_buffer_size = getParamOrThrow<double>("~matlogger_buffer_size");

    _is_first_jnt_passive = getParamOrThrow<bool>("~is_first_jnt_passive"); 

    _verbose = getParamOrThrow<bool>("~verbose");

}

void IqModelCalibRt::is_sim(std::string sim_string = "sim")
{
    XBot::Context ctx;
    auto& pm = ctx.paramManager();
    _hw_type = pm.getParamOrThrow<std::string>("/xbot_internal/hal/hw_type");

    size_t sim_found = _hw_type.find(sim_string);

    if (sim_found != std::string::npos) { // we are running the plugin in simulation

        _is_sim = true;
    }
    else // we are running on the real robot
    {
        _is_sim = false;
    }

}

void IqModelCalibRt::create_ros_api()
{
    /*  Create ros api server */
    RosServerClass::Options opt;
    opt.tf_prefix = getParamOr<std::string>("~tf_prefix", "mr");
    opt.ros_namespace = getParamOr<std::string>("~ros_ns", "iq_model_calib_rt");
}

void IqModelCalibRt::update_state()
{    
    // "sensing" the robot
    _robot->sense();

    // Getting robot state
    _robot->getJointPosition(_q_p_meas);
    _robot->getMotorVelocity(_q_p_dot_meas);  
    _robot->getJointEffort(_tau_meas);

}

void IqModelCalibRt::init_dump_logger()
{

    // // Initializing logger for debugging
    MatLogger2::Options opt;
    opt.default_buffer_size = _matlogger_buffer_size; // set default buffer size
    opt.enable_compression = true; // enable ZLIB compression
    if (_dump_mat_suffix == std::string(""))
    { // if empty, then dump to tmp with plugin name
        _dump_logger = MatLogger2::MakeLogger("/tmp/IqModelCalibRt", opt); // date-time automatically appended
    }
    else
    {
        if (!_is_sim)
        {
            _dump_logger = MatLogger2::MakeLogger(_mat_path + std::string("test_") + _dump_mat_suffix, opt); // date-time automatically appended
        }
        else
        {
            _dump_logger = MatLogger2::MakeLogger(_mat_path + std::string("sim_") + _dump_mat_suffix, opt); // date-time automatically appended
        }
        
    }

    _dump_logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);

    _dump_logger->add("plugin_dt", _plugin_dt);
    _dump_logger->add("is_sim", int(_is_sim));


    _dump_logger->create("q_p_meas", _n_jnts_robot), 1, _matlogger_buffer_size;
    _dump_logger->create("q_p_dot_meas", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("tau_meas", _n_jnts_robot, 1, _matlogger_buffer_size);

}

void IqModelCalibRt::add_data2dump_logger()
{

    _dump_logger->add("q_p_meas", _q_p_meas);
    _dump_logger->add("q_p_dot_meas", _q_p_dot_meas);
    _dump_logger->add("tau_meas", _tau_meas);

}

void IqModelCalibRt::add_data2bedumped()
{
    
    // _dump_logger->add("plugin_time", _loop_time)

    if (_reduce_dumped_sol_size)
    {  // only adding data when replaying trajectory to save memory

        if (_traj_started && !_traj_finished)
        { // trajectory is being published
            
            if (_sample_index <= (_traj.get_n_nodes() - 1))
            {

                add_data2dump_logger();

            }

        }
    }
    else
    { // dump data at each loop cycle

         add_data2dump_logger();

    }
    
    

}

void IqModelCalibRt::init_nrt_ros_bridge()
{    
    ros::NodeHandle nh(getName());

    _ros = std::make_unique<RosSupport>(nh);

    /* Service server */
    _jump_now_srv = _ros->advertiseService(
        "my_jump_now",
        &IqModelCalibRt::on_jump_msg_rcvd,
        this,
        &_queue);

    /* Subscribers */
    _aux_signals_sub = _ros->subscribe("/xbotcore/link_state/base_link/pose",
                                &IqModelCalibRt::on_aux_signal_received,
                                this,
                                1,  // queue size
                                &_queue);

    /* Publishers */
    awesome_leg::MatReplayerStatus replay_st_prealloc;
    replay_st_prealloc.approach_traj_finished = false;
    replay_st_prealloc.traj_finished = false;

    _replay_status_pub = _ros->advertise<awesome_leg::MatReplayerStatus>(
        "replay_status_node", 1, replay_st_prealloc);


}

void MatReplayerRt::on_aux_signal_received()
{

}

bool IqModelCalibRt::on_initialize()
{ 
    std::string sim_flagname = "sim";

    is_sim(sim_flagname); // see if we are running a simulation

    get_params_from_config(); // load params from yaml file

    _plugin_dt = getPeriodSec();

    _n_jnts_robot = _robot->getJointNum();

    init_vars();

    init_nrt_ros_bridge();

    create_ros_api();

    return true;
    
}

void IqModelCalibRt::starting()
{

    init_dump_logger(); // needs to be here

    reset_flags(); // reset flags, just in case

    init_clocks(); // initialize clocks timers

    update_state(); // read current jnt positions and velocities

    start_completed(); // Move on to run()

}

void IqModelCalibRt::run()
{  

    update_state(); // update all necessary states

    _queue.run();

    add_data2dump_logger(); // add data to the logger

    update_clocks(); // last, update the clocks (loop + any additional one)

    if (_is_first_run)
    { // next control loops are aware that it is not the first control loop
        _is_first_run = !_is_first_run;
    }

}

void IqModelCalibRt::on_stop()
{
    // Destroy logger and dump .mat file (will be recreated upon plugin restart)
    _dump_logger.reset();
}

void IqModelCalibRt::stopping()
{
    stop_completed();
}

void IqModelCalibRt::on_abort()
{
    // Destroy logger and dump .mat file
    _dump_logger.reset();
}

void IqModelCalibRt::on_close()
{
    jinfo("Closing IqModelCalibRt");
}

XBOT2_REGISTER_PLUGIN(IqModelCalibRt, iq_model_calib_rt)

