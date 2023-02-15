#ifndef RECOV_ENERGY_REACHED_H
#define RECOV_ENERGY_REACHED_H

#include <behaviortree_cpp_v3/condition_node.h>
#include <xbot2/journal/journal.h>

#include <xbot2/xbot2.h>
#include <xbot2/ros/ros_support.h>

#include <cartesian_interface/sdk/rt/LockfreeBufferImpl.h>
#include <cartesian_interface/ros/RosServerClass.h>

#include <awesome_leg/RegPowStatus.h>

using namespace XBot;

namespace BT
{

    class RecovEnergyReached : public ConditionNode, public Task
    {
        public:

            // You must provide the function to call when tick() is invoked
            RecovEnergyReached(const std::string& name, const NodeConfiguration& config);
            RecovEnergyReached(const std::string& name, const NodeConfiguration& config, double& recov_energy_thresh);

            ~RecovEnergyReached() override = default;

            static PortsList providedPorts(); // this is required to be a static method

        private:

            bool _verbose = false;

            int _queue_size = 1;

            std::string _rec_evergy_topicname = "reg_pow_node_iq_model";

            std::string _rec_energy_pluginname = "bus_power_rt_iq_model";

            double _recov_energy_thresh = 3 * 300.0; // [J]

            awesome_leg::RegPowStatus _reg_pow_status;

            SubscriberPtr<awesome_leg::RegPowStatus> _reg_pow_sub;

            NodeStatus tick() override;

    };

}

#endif // RECOV_ENERGY_REACHED_H
