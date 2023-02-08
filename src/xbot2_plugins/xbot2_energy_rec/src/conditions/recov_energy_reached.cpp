#include "recov_energy_reached.h"

#include "../utils_defs.hpp"

using namespace BT;

RecovEnergyReached::RecovEnergyReached(const std::string& name, const NodeConfiguration& config) :
    ConditionNode(name, config),
    Task(name + "_bt_leaf", ""){

    setRegistrationID(name);

    // lambda to define callback
    auto reg_energy_callback = [this](const awesome_leg::RegPowStatus& msg)
    {

        _reg_pow_status.recov_energy = msg.recov_energy;

    };

    _reg_pow_sub = subscribe<awesome_leg::RegPowStatus>("/" + _rec_energy_pluginname + "/" + _rec_evergy_topicname,
                            reg_energy_callback,
                            _queue_size);

    _reg_pow_status.recov_energy = 0.0;// fif we tick the node without having received an update
    // from the callback, we assume the recovered energy to be 0
};

PortsList RecovEnergyReached::providedPorts()
{

  return { OutputPort<bool>("recov_energy_reached") };

}

NodeStatus RecovEnergyReached::tick()
{

    setOutput("recov_energy_reached", true);

    _reg_pow_sub->run();

//    std::cout << Colors::kGreen << "ticking RecovEnergyReached" << Colors::kEndl << std::endl;

    NodeStatus result = _reg_pow_status.recov_energy > _recov_energy_thresh? NodeStatus::SUCCESS : NodeStatus::FAILURE;

    return result;

}
