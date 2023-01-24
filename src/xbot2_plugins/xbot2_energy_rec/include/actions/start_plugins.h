#ifndef START_PLUGINS_H
#define START_PLUGINS_H

#include <behaviortree_cpp_v3/action_node.h>

using namespace BT;

namespace rt_BT
{

    class StartPlugins : public SyncActionNode
    {
        public:

        StartPlugins(const std::string& name) : SyncActionNode(name, {})
        {
            setRegistrationID("StartPlugins");
        }

        private:

            virtual NodeStatus tick() override
            {
                return NodeStatus::FAILURE;
            }

    };

}
#endif // START_PLUGINS_H
