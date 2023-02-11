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
        if(_verbose)
        {
            std::cout << Colors::kGreen << "rec energy callback" << Colors::kEndl << std::endl;

        }
        _reg_pow_status.recov_energy_tot = msg.recov_energy_tot;

    };

    _reg_pow_sub = subscribe<awesome_leg::RegPowStatus>("/" + _rec_energy_pluginname + "/" + _rec_evergy_topicname,
                            reg_energy_callback,
                            _queue_size);

    _reg_pow_status.recov_energy_tot = 0.0;// fif we tick the node without having received an update
    // from the callback, we assume the recovered energy to be 0

};

RecovEnergyReached::RecovEnergyReached(const std::string& name, const NodeConfiguration& config, double& recov_energy_thresh) :
    ConditionNode(name, config),
    Task(name + "_bt_leaf", "")
{
    RecovEnergyReached(name, config);

    _recov_energy_thresh = recov_energy_thresh;

}

PortsList RecovEnergyReached::providedPorts()
{

  return { OutputPort<bool>("recov_energy_reached") };

}

NodeStatus RecovEnergyReached::tick()
{

    _reg_pow_sub->run();

    if(_verbose)
    {
        std::cout << Colors::kGreen << "ticking RecovEnergyReached. Rec. energy: " << Colors::kBlue <<_reg_pow_status.recov_energy_tot << Colors::kEndl << std::endl;

    }

    NodeStatus result = _reg_pow_status.recov_energy_tot > _recov_energy_thresh? NodeStatus::SUCCESS : NodeStatus::FAILURE;

    setOutput("recov_energy_reached", _reg_pow_status.recov_energy_tot);

    return result;

}
