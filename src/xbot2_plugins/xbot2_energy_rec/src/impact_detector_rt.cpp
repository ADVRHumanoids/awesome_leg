#include "impact_detector_rt.h"

#include "utils_defs.hpp"
#include <iostream>

#include <math.h>

Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

void ImpactDetectorRt::init_clocks()
{
    _loop_time = 0.0; // reset loop time clock
}

void ImpactDetectorRt::reset_flags()
{

}

void ImpactDetectorRt::update_clocks()
{
    // Update timer(s)
    _loop_time += _plugin_dt;

    // Reset timers, if necessary
    if (_loop_time >= _loop_timer_reset_time)
    {
        _loop_time = _loop_time - _loop_timer_reset_time;
    }

}

void ImpactDetectorRt::get_params_from_config()
{

//    _queue_size = getParamOrThrow<int>("~queue_size");

    _verbose = getParamOrThrow<bool>("~verbose");

    _mat_path = getParamOrThrow<std::string>("~mat_path");
//    _dump_mat_suffix = getParamOrThrow<std::string>("~dump_mat_suffix");
//    _matlogger_buffer_size = getParamOrThrow<double>("~matlogger_buffer_size");

//    _impact_state_topicname = getParamOrThrow<std::string>("~topicname");

    _use_gz_ground_truth =getParamOrThrow<bool>("~use_gz_truth");

    _use_contact2trigger_pow = getParamOrThrow<bool>("~use_contact2trigger_pow");

}

void ImpactDetectorRt::is_sim(std::string sim_string = "sim")
{
    XBot::Context ctx;
    auto& pm = ctx.paramManager();
    _hw_type = pm.getParamOrThrow<std::string>("/xbot/hal/hw_type");

    size_t sim_found = _hw_type.find(sim_string);

    if (sim_found != std::string::npos) { // we are running the plugin in simulation

        _is_sim = true;
    }
    else // we are running on the real robot
    {
        _is_sim = false;
    }

}

void ImpactDetectorRt::is_dummy(std::string dummy_string = "dummy")
{
    XBot::Context ctx;
    auto& pm = ctx.paramManager();
    _hw_type = pm.getParamOrThrow<std::string>("/xbot/hal/hw_type");

    size_t dummy_found = _hw_type.find(dummy_string);


    if (dummy_found != std::string::npos) { // we are running the plugin in dummy mode

        _is_dummy = true;
    }
    else // we are running on the real robot
    {
        _is_dummy = false;
    }

}

void ImpactDetectorRt::init_dump_logger()
{

    // // Initializing logger for debugging
    MatLogger2::Options opt;
    opt.default_buffer_size = _matlogger_buffer_size; // set default buffer size
    opt.enable_compression = true; // enable ZLIB compression
    if (_dump_mat_suffix == std::string(""))
    { // if empty, then dump to tmp with plugin name
        _dump_logger = MatLogger2::MakeLogger("/tmp/ImpactDetectorRt", opt); // date-time automatically appended
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
    _dump_logger->add("is_dummy", int(_is_dummy));
    _dump_logger->add("verbose", int(_verbose));

}

void ImpactDetectorRt::add_data2dump_logger()
{


}


void ImpactDetectorRt::add_data2bedumped()
{

    add_data2dump_logger();

}

void ImpactDetectorRt::init_communications()
{
    ros::NodeHandle nh(getName());

    _ros = std::make_unique<RosSupport>(nh);

    // regenerative energy monitoring
    _start_monitoring_msg.obj.monitor_energy = true;
    _stop_monitoring_msg.obj.monitor_energy = false;
    _stop_monitoring_msg.obj.reset_energy = true;

    _asynch_servicepath = _async_service_pattern + _pow_monitor_pluginname + "/" + _pow_monitor_servicename + "/request";

    _monitoring_pow_switch_pub = advertise<SetRegEnergyMonitoringStatusRequest>(_asynch_servicepath);

    // base estimation
    auto base_est_callback = [this](const BaseEstStatus& msg)
    {

        if(_verbose)
        {
            std::cout << Colors::kMagenta << "\nImpactDetectorRt: received BaseEstStatus msg\n" << Colors::kEndl << std::endl;
        }

        _base_est_status.contact_state = msg.contact_state;
        _base_est_status.contact_ground_truth = msg.contact_ground_truth;

        bool contact_prev;
        bool contact_now;

        if(_use_gz_ground_truth && (_is_sim || _is_dummy))
        { // we use gazebo ground truth

            contact_prev = _base_est_status_prev.contact_ground_truth;
            contact_now = _base_est_status.contact_ground_truth;

        }
        else
        { // we use the base estimation contact state
            contact_prev = _base_est_status_prev.contact_state;
            contact_now = _base_est_status.contact_state;
        }

        if(!contact_prev && contact_now)
        { // impact
            _impact_status_msg.contact =  false;
            _impact_status_msg.flight = false;
            _impact_status_msg.impact = true;
            _impact_status_msg.takeoff = false;
        }

        if(contact_prev && !contact_now)
        { // takeoff
            _impact_status_msg.contact =  false;
            _impact_status_msg.flight = false;
            _impact_status_msg.impact = false;
            _impact_status_msg.takeoff = true;
        }

        if(contact_prev && contact_now)
        { // contact
            _impact_status_msg.contact =  true;
            _impact_status_msg.flight = false;
            _impact_status_msg.impact = false;
            _impact_status_msg.takeoff = false;
        }

        if(!contact_prev && !contact_now)
        { // flight
            _impact_status_msg.contact =  false;
            _impact_status_msg.flight = true;
            _impact_status_msg.impact = false;
            _impact_status_msg.takeoff = false;
        }

    };

    _base_est_status_sub = subscribe<BaseEstStatus>("/" + _base_est_pluginname + "/" + _base_est_status_topicname,
                            base_est_callback,
                            _queue_size);

    // jump replay status
    auto jump_replay_callback = [this](const MatReplayerStatus& msg)
    {

        if(_verbose)
        {
            std::cout << Colors::kBlue << "\nImpactDetectorRt: received MatReplayerStatus msg\n" << Colors::kEndl << std::endl;

        }

        _jump_replay_status.approach_traj_started = msg.approach_traj_started;
        _jump_replay_status.traj_started = msg.traj_started;
        _jump_replay_status.imp_traj_started = msg.imp_traj_started;
        _jump_replay_status.landing_config_reached = msg.landing_config_reached;

        _jump_replay_status.approach_traj_finished = msg.approach_traj_finished;
        _jump_replay_status.traj_finished = msg.traj_finished;
        _jump_replay_status.imp_traj_finished = msg.imp_traj_finished;
        _jump_replay_status.landing_config_started = msg.landing_config_started;


    };

    _jump_replay_status_sub = subscribe<MatReplayerStatus>("/" + _jump_replay_pluginname + "/" + _jump_replay_topicname,
                            jump_replay_callback,
                            _queue_size);

    // impact state publisher

    _impact_status_pub = _ros->advertise<awesome_leg::ImpactStatus>(
            _impact_state_topicname.c_str(), 1);

}

void ImpactDetectorRt::trigger_reg_pow_monitor()
{

    // standard operation: to trigger the reg pow monitor we look at the contact status (either from Gazebo or the base estimator)
    if(_impact_status_msg.contact &&
            _jump_replay_status.imp_traj_started && _use_contact2trigger_pow)
    { // ramping impedance with contact --> we don't monitor the energy
        _monitoring_pow_switch_pub->publish(_stop_monitoring_msg);

    }
    if(_impact_status_msg.contact &&
            _jump_replay_status.approach_traj_started && _use_contact2trigger_pow)
    { // approach trajectory with contact --> we don't monitor the energy
        _monitoring_pow_switch_pub->publish(_stop_monitoring_msg);

    }
    if(_impact_status_msg.contact &&
            _jump_replay_status.traj_started && _use_contact2trigger_pow)
    { // during jump trajectory with contact --> we don't monitor the energy
        _monitoring_pow_switch_pub->publish(_stop_monitoring_msg);

    }
    if(_impact_status_msg.takeoff && _use_contact2trigger_pow)
    { // at takeoff --> we don't monitor the energy in any case
        _monitoring_pow_switch_pub->publish(_stop_monitoring_msg);

    }
    if(_impact_status_msg.flight && _use_contact2trigger_pow)
    { // flight phase --> we don't monitor the energy in any case
        _monitoring_pow_switch_pub->publish(_stop_monitoring_msg);

    }
    if(_impact_status_msg.impact && !_jump_replay_status.traj_finished && _use_contact2trigger_pow)
    { // impact instant, but we didn't finish replaying jump trajectory -> we don't monitor the energy
        _monitoring_pow_switch_pub->publish(_stop_monitoring_msg);

    }

    if(_impact_status_msg.impact && _jump_replay_status.landing_config_reached && _use_contact2trigger_pow)
    { // impact instant, after replaying jump trajectory -> we trigger the monitoring of reg power
        _monitoring_pow_switch_pub->publish(_start_monitoring_msg);

        std::cout << Colors::kBlue << "\n ImpactDetectorRt: sending message to start regenerative power monitoring \n" << Colors::kEndl << std::endl;

    }

    if(_impact_status_msg.contact  && _jump_replay_status.imp_traj_started && !_jump_replay_status_prev.imp_traj_started && _use_contact2trigger_pow)
    { // after landing and just before restarting the jump sequence --> we stop monitoring the power
        _monitoring_pow_switch_pub->publish(_stop_monitoring_msg);

        std::cout << Colors::kBlue << "\n ImpactDetectorRt: sending message to stop regenerative power monitoring \n" << Colors::kEndl << std::endl;

    }

    // "acroccata" operation: to trigger the reg pow monitor we look at the contact status (either from Gazebo or the base estimator)

    if(_jump_replay_status.landing_config_reached && !_use_contact2trigger_pow)
    { // we suppose that no energy is recovered right after the trajectory replay (more or less an approximation) -> we trigger the monitoring of reg power
        _monitoring_pow_switch_pub->publish(_start_monitoring_msg);

        std::cout << Colors::kBlue << "\n ImpactDetectorRt: sending message to start regenerative power monitoring \n" << Colors::kEndl << std::endl;

    }
    if(_jump_replay_status.imp_traj_started && !_jump_replay_status_prev.imp_traj_started && !_use_contact2trigger_pow)
    { // after landing and just before restarting the jump sequence --> we stop monitoring the power
        _monitoring_pow_switch_pub->publish(_stop_monitoring_msg);

        std::cout << Colors::kBlue << "\n ImpactDetectorRt: sending message to stop regenerative power monitoring \n" << Colors::kEndl << std::endl;

    }

}

bool ImpactDetectorRt::on_initialize()
{
    std::string sim_flagname = "sim";
    is_sim(sim_flagname); // see if we are running a simulation

    std::string dummy_flagname = "dummy";
    is_dummy(dummy_flagname); // see if we are running in dummy mode

    get_params_from_config(); // load params from yaml file

    _plugin_dt = getPeriodSec();

    init_communications();

    return true;

}

void ImpactDetectorRt::starting()
{

    init_dump_logger(); // needs to be here

    reset_flags(); // reset flags, just in case

    init_clocks(); // initialize clocks timers

    start_completed(); // Move on to run()


}

void ImpactDetectorRt::run()
{
    _base_est_status_prev = _base_est_status; // we first get the latest base estimation state
    _jump_replay_status_prev = _jump_replay_status;

    // we then update the necessary states by processing the callbacks
//    _queue.run(); // process service callbacks
    _base_est_status_sub->run();
    _jump_replay_status_sub->run();

    trigger_reg_pow_monitor();

    // publish impact state
    _impact_status_pub->publish(_impact_status_msg);

    add_data2bedumped(); // add data to the logger

    update_clocks(); // last, update the clocks (loop + any additional one)

}

void ImpactDetectorRt::on_stop()
{
    // Destroy logger and dump .mat file (will be recreated upon plugin restart)
    _dump_logger.reset();
}

void ImpactDetectorRt::stopping()
{
    stop_completed();
}

void ImpactDetectorRt::on_abort()
{
    // Destroy logger and dump .mat file
    _dump_logger.reset();
}

void ImpactDetectorRt::on_close()
{
    jinfo("Closing ImpactDetectorRt");
}

XBOT2_REGISTER_PLUGIN(ImpactDetectorRt, impact_detector_rt)
