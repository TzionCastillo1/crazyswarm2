/**
 * @file crazyflie_sitl_controller.cc
 * @author Tzion Castillo (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-03-04
 * 
 * A plugin to connect crazyflie SITL with Ignition Gazebo
 * 
 */


#include "crazyflie_sitl_controller.hh"

#include <ignition/plugin/Register.hh>

#include <ignition/msgs/actuators.pb.h>
#include "ignition/gazebo/components/Actuators.hh"
#include <ignition/gazebo/Util.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <netinet/in.h>
#include <sys/socket.h>

#define PORT 4560


using namespace crazyflie_sitl_controller;

IGNITION_ADD_PLUGIN(
    crazyflie_sitl_controller::CrazyflieSITLController,
    ignition::gazebo::System,
    crazyflie_sitl_controller::CrazyflieSITLController::ISystemConfigure,
    crazyflie_sitl_controller::CrazyflieSITLController::ISystemPreUpdate

)

using namespace crazyflie_sitl_controller;

CrazyflieSITLController::CrazyflieSITLController()
{
    int server_fd, new_socket, valread;
    struct sockaddr_in address;
    int opt=1;
    int addrlen = sizeof(address);

    char buffer[1024] = {0};

    if((server_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }
}

CrazyflieSITLController::~CrazyflieSITLController()
{
}

void CrazyflieSITLController::Configure(const ignition::gazebo::Entity &_entity,
                                        const std::shared_ptr<const sdf::Element> &_sdf,
                                        ignition::gazebo::EntityComponentManager &_ecm,
                                        ignition::gazebo::EventManager & /*_event_Mgr*/)

this->model = ignition::gazebo::Model(_entity);
this->motorCommands.mutable_velocity()->Resize(4,0);

_ecm.CreateComponent(this->model.Entity(),
                     ignition::gazebo::components::Actuators(this->motorCommands));

double past_time;
void CrazyflieSITLController::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                                        ignitino::gazebo::EntityComponentManager &_ecm)
{
    auto current_time_ms = std::chrono:duration_cast<std::chrono::milliseconds>(
        _info.simTime).count();
    
    double current_time = (double) current_time_ms;
    double dt = current_time - this->past_time;
    this->past_time = current_time;

    

    /** TODO: Add socket connection to SIL interface using port 4560 to follow PX4 standards for SITL simulation
     *  messages to send:
     *  SIMULATED_IMU,
     *  ACTUAL_STATE,
     * 
     *  messages to receive:
     *  ACTUATOR_CONTROLS
    */



}

int main(int argc, char * argv[])
{

    return 0;
}

