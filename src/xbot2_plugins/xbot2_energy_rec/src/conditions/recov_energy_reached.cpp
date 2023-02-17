#include "recov_energy_reached.h"

#include "../utils_defs.hpp"

using namespace BT;

//** RecovEnergyReached **//

RecovEnergyReached::RecovEnergyReached(const std::string& name, const NodeConfiguration& config) :
    ConditionNode(name, config),
    Task(name + "_bt_leaf", ""){

    setRegistrationID(name);

    // lambda to define callback
    auto reg_energy_callback = [this](const awesome_leg::EstRegPowStatus& msg)
    {
        if(_verbose)
        {
            std::cout << Colors::kGreen << "rec energy callback" << Colors::kEndl << std::endl;

        }
        _reg_pow_status.recov_energy_tot = msg.recov_energy_tot;

    };

    _reg_pow_sub = subscribe<awesome_leg::EstRegPowStatus>("/" + _rec_energy_pluginname + "/" + _rec_evergy_topicname,
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

//** RecovEnergyReachedMeas **//

RecovEnergyReachedMeas::RecovEnergyReachedMeas(const std::string& name, const NodeConfiguration& config) :
    ConditionNode(name, config),
    Task(name + "_bt_leaf", ""){

    setRegistrationID(name);

    // lambda to define callback
    auto reg_energy_callback = [this](const awesome_leg::MeasRegPowStatus& msg)
    {
        if(_verbose)
        {
            std::cout << Colors::kGreen << "rec energy callback" << Colors::kEndl << std::endl;

        }
        _reg_pow_status.reg_energy = msg.reg_energy;

    };

    _reg_pow_sub = subscribe<awesome_leg::MeasRegPowStatus>("/" + _rec_energy_pluginname + "/" + _rec_evergy_topicname,
                            reg_energy_callback,
                            _queue_size);

    _reg_pow_status.reg_energy = 0.0;// fif we tick the node without having received an update
    // from the callback, we assume the recovered energy to be 0

};

RecovEnergyReachedMeas::RecovEnergyReachedMeas(const std::string& name, const NodeConfiguration& config, double& recov_energy_thresh) :
    ConditionNode(name, config),
    Task(name + "_bt_leaf", "")
{
    RecovEnergyReached(name, config);

    _recov_energy_thresh = recov_energy_thresh;

}

PortsList RecovEnergyReachedMeas::providedPorts()
{

  return { OutputPort<bool>("recov_energy_reached") };

}

NodeStatus RecovEnergyReachedMeas::tick()
{

    _reg_pow_sub->run();

    if(_verbose)
    {
        std::cout << Colors::kGreen << "ticking RecovEnergyReachedMeas. Rec. energy: " << Colors::kBlue <<_reg_pow_status.reg_energy << Colors::kEndl << std::endl;

    }

    NodeStatus result = _reg_pow_status.reg_energy > _recov_energy_thresh? NodeStatus::SUCCESS : NodeStatus::FAILURE;

    setOutput("recov_energy_reached", _reg_pow_status.reg_energy);

    return result;

}
