#include "mep3_simulation/mep3_webots_hardware_interface.hpp"

#include <cmath>
#include <cstring>
#include <iostream>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace mep3_simulation
{

    void Mep3WebotsHardwareInterface::init(webots_ros2_driver::WebotsNode *node, const hardware_interface::HardwareInfo &info)
    {
        node_ = node;

        for (hardware_interface::ComponentInfo component : info.gpios)
        {
            Pin pin;
            pin.name = component.name;
            pins_.emplace_back(pin);
        }
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Mep3WebotsHardwareInterface::on_init(const hardware_interface::HardwareInfo &info)
    {
        if (hardware_interface::SystemInterface::on_init(info) !=
            CallbackReturn::SUCCESS)
        {
            return CallbackReturn::ERROR;
        }

        return CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Mep3WebotsHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
    {
        return CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Mep3WebotsHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
    {
        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> Mep3WebotsHardwareInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> interfaces;
        return interfaces;
    }

    std::vector<hardware_interface::CommandInterface> Mep3WebotsHardwareInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> interfaces;

        for (Pin &pin : pins_)
            interfaces.emplace_back(hardware_interface::CommandInterface(pin.name, "output", &(pin.output)));

        return interfaces;
    }

    hardware_interface::return_type Mep3WebotsHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type Mep3WebotsHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        for (Pin &pin : pins_) {
            if (pin.output != pin.previous_output)
                RCLCPP_WARN(node_->get_logger(), "Pin %s writes %lf", pin.name.c_str(), pin.output);
            pin.previous_output = pin.output;
        }

        return hardware_interface::return_type::OK;
    }
} // namespace mep3_simulation

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mep3_simulation::Mep3WebotsHardwareInterface,
                       webots_ros2_control::Ros2ControlSystemInterface)
