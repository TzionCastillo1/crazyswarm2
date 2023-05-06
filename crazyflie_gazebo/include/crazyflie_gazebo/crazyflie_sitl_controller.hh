
#ifndef SYSTEM_PLUGIN_CRAZYFLIESITLCONTROLLER_HH_
#define SYSTEM_PLUGIN_CRAZYFLIESITLCONTROLLER_HH_

#include <ignition/gazebo/System.hh>
#include <memory>
#include <string>
#include <ignition/msgs.hh>
#include <ignition/gazebo/components/Component.hh>
#include <ignition/gazebo/config.hh>
#include "ignition/gazebo/Model.h"
#include <ignition/gazebo/msgs/twist.pb.h>
#include <ignition/transport11/Node.hh>

namespace crazyflie_sitl_controller
{
    class CrazyflieSITLController:
        public ignition::gazebo::System,
        public ignition::gazebo::ISystemConfigure,
        public ignition::gazebo::ISystemPreUpdate
    {
        public: CrazyflieSITLController();

        public: ~CrazyflieSITLController() override;

        public: void Configure(const ignition::gazebo::Entity &_entity,
                                const std::shared_ptr<const sdf::Element> &,
                                ignition::gazebo::EntityComponentManager &_ecm,
                                ignition::gazebo::EventManager &) override;

        public: void PreUpdate(const ignitino::gazebo::UpdateInfo &_info,
                                ignition::gazebo::EntityComponentManager &_ecm) override;

        private: ignition::msgs::Actuators motorCommands;
        private: ignition::gazebo::Model model{ignition::gazebo::kNullEntity};

        private: double pastTime;

        private: ignitino::transport::Node node;         
    };
}

#endif


