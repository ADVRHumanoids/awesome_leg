#ifndef ARE_PLUGINS_RUNNING_H
#define ARE_PLUGINS_RUNNING_H

#include <behaviortree_cpp_v3/condition_node.h>
#include <behaviortree_cpp_v3/leaf_node.h>

namespace BT
{

    class ArePluginsRunning : public ConditionNode
    {
    public:

        ArePluginsRunning() = default;

        // You must provide the function to call when tick() is invoked
        ArePluginsRunning(const std::string& name) : ConditionNode(name, {}){

            setRegistrationID(name);
        };

        ~ArePluginsRunning() override = default;

    private:

        virtual NodeStatus tick() override;

    };

}

#endif // ARE_PLUGINS_RUNNING_H
