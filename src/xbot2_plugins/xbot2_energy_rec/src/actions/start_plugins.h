#ifndef START_PLUGINS_H
#define START_PLUGINS_H

#include <behaviortree_cpp_v3/action_node.h>
#include <xbot2/journal/journal.h>

namespace BT
{

    class StartPlugins : public AsyncActionNode
    { // we are working with rt plugins, so we should minimize blocking code
      // that's why we employ asynchronous action nodes

        public:

            StartPlugins(const std::string& name) : AsyncActionNode(name, {})
            {
                setRegistrationID(name);
            }

        private:

            NodeStatus tick() override;

    };

}

#endif // START_PLUGINS_H
