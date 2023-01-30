#ifndef ARE_PLUGINS_RUNNING_H
#define ARE_PLUGINS_RUNNING_H

#include <behaviortree_cpp_v3/condition_node.h>
#include <xbot2/journal/journal.h>
#include "../utils_defs.hpp"

namespace BT
{

    class ArePluginsRunning : public ConditionNode
    {
        public:

            // You must provide the function to call when tick() is invoked
            ArePluginsRunning(const std::string& name, const NodeConfiguration& config);

            ~ArePluginsRunning() override = default;

            static PortsList providedPorts(); // this is required to be a static method

        private:

            NodeStatus tick() override;

    };

}

#endif // ARE_PLUGINS_RUNNING_H
