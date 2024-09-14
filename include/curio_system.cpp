#include "curio_one/curio_system.hpp"
#include "sensor_msgs/msg/range.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/sensor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp" 

namespace curio_one
{
CallbackReturn CurioBotSystemHardware::on_init(const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
        return CallbackReturn::ERROR;
    }

    cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
    cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
    cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
    cfg_.device = info_.hardware_parameters["device"];
    cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
    cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
    cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);
    cfg_.battery_name = info_.hardware_parameters["battery_name"];


    wheel_r_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);
    wheel_l_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
    battery_.setup(cfg_.battery_name); 

    // Initialize publisher for TOF sensor range data
    // range_ = node_->create_publisher<sensor_msgs::msg::Range>("tof_range", 10);

    //Set up sensor
    if (!info_.sensors.empty()) {
        const auto& sensor = info_.sensors[0]; // Assuming only one TOF sensor is available
        cfg_.sensor_name = sensor.name;
        cfg_.sensor1_name = sensor.name + "_1";  // Unique name for first additional sensor
        cfg_.sensor2_name = sensor.name + "_2";  // Unique name for second additional sensor
        cfg_.sensor4_name = sensor.name + "_4";  // Unique name for third additional sensor
        cfg_.sensor5_name = sensor.name + "_5";  // Unique name for fouth additional sensor
        cfg_.sensor6_name = sensor.name + "_6";  // Unique name for fifth additional sensor


    } else {
        // Handle the case when no sensors are available
        RCLCPP_ERROR(rclcpp::get_logger("CurioBotSystemHardware"), "No sensors available!");
        return CallbackReturn::ERROR;
    }

    sensor_n_.setup(cfg_.sensor_name);
    sensor_l_.setup(cfg_.sensor1_name);
    sensor_r_.setup(cfg_.sensor2_name);

    sensor_n1_.setup(cfg_.sensor4_name);
    sensor_l1_.setup(cfg_.sensor5_name);
    sensor_r1_.setup(cfg_.sensor6_name);


    for (const hardware_interface::ComponentInfo & joint : info_.joints)
    {
        // CurioBotSystem has exactly two states and one command interface on each joint
        if (joint.command_interfaces.size() != 1)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("CurioBotSystemHardware"),
                "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                joint.command_interfaces.size());
            return CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("CurioBotSystemHardware"),
                "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
                joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
            return CallbackReturn::ERROR;
        }

        if (joint.state_interfaces.size() != 2)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("CurioBotSystemHardware"),
                "Joint '%s' has %zu state interface. 3 expected.", joint.name.c_str(),
                joint.state_interfaces.size());
            return CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("CurioBotSystemHardware"),
                "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
                joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("CurioBotSystemHardware"),
                "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
                joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
            return CallbackReturn::ERROR;
        }

        // if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT)
        // {
        //   RCLCPP_FATAL(
        //     rclcpp::get_logger("CurioBotSystemHardware"),
        //     "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        //     joint.state_interfaces[2].name.c_str(), hardware_interface::HW_IF_EFFORT);
        //   return CallbackReturn::ERROR;
        // }
    }

    for (const hardware_interface::ComponentInfo & sensors : info_.sensors)
    {
        if (sensors.state_interfaces.size() != 6)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("CurioBotSystemHardware"),
                "Joint '%s' has %zu state interface. 6 expected.", sensors.name.c_str(),
                sensors.state_interfaces.size());
            return CallbackReturn::ERROR;
        }

        if (sensors.state_interfaces[0].name != "Sensor")
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("CurioBotSystemHardware"),
                "Joint '%s' have '%s' as first state interface. '%s' expected.", sensors.name.c_str(),
                sensors.state_interfaces[0].name.c_str(), "Sensor");

            return CallbackReturn::ERROR;
        }

        if (sensors.state_interfaces[1].name != "Sensor1")
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("CurioBotSystemHardware"),
                "Joint '%s' have '%s' as second state interface. '%s' expected.", sensors.name.c_str(),
                sensors.state_interfaces[1].name.c_str(), "Sensor1");

            return CallbackReturn::ERROR;
        }

        if (sensors.state_interfaces[2].name != "Sensor2")
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("CurioBotSystemHardware"),
                "Joint '%s' have '%s' as third   state interface. '%s' expected.", sensors.name.c_str(),
                sensors.state_interfaces[2].name.c_str(), "Sensor2");

            return CallbackReturn::ERROR;
        }

        if (sensors.state_interfaces[3].name != "Sensor4")
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("CurioBotSystemHardware"),
                "Joint '%s' have '%s' as second state interface. '%s' expected.", sensors.name.c_str(),
                sensors.state_interfaces[3].name.c_str(), "Sensor4");

            return CallbackReturn::ERROR;
        }

        if (sensors.state_interfaces[4].name != "Sensor5")
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("CurioBotSystemHardware"),
                "Joint '%s' have '%s' as third   state interface. '%s' expected.", sensors.name.c_str(),
                sensors.state_interfaces[4].name.c_str(), "Sensor5");

            return CallbackReturn::ERROR;
        }

        if (sensors.state_interfaces[5].name != "Sensor6")
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("CurioBotSystemHardware"),
                "Joint '%s' have '%s' as third   state interface. '%s' expected.", sensors.name.c_str(),
                sensors.state_interfaces[5].name.c_str(), "Sensor6");

            return CallbackReturn::ERROR;
        }
        // if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT)
        // {
        //   RCLCPP_FATAL(
        //     rclcpp::get_logger("CurioBotSystemHardware"),
        //     "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        //     joint.state_interfaces[2].name.c_str(), hardware_interface::HW_IF_EFFORT);
        //   return CallbackReturn::ERROR;
        // }
    }

    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> CurioBotSystemHardware::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    // Export state interfaces for the left wheel
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        wheel_l_.name, hardware_interface::HW_IF_POSITION, &wheel_l_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.vel));

    // Export state interfaces for the right wheel
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        wheel_r_.name, hardware_interface::HW_IF_POSITION, &wheel_r_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.vel));

    // Export state interface for the TOF sensor
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "tof_joint", "Sensor", &sensor_n_.reading));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "tof_joint", "Sensor1", &sensor_l_.reading));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "tof_joint", "Sensor2", &sensor_r_.reading));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "tof_joint", "Sensor4", &sensor_n1_.reading));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "tof_joint", "Sensor5", &sensor_l1_.reading));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "tof_joint", "Sensor6", &sensor_r1_.reading));

    // Export state interface for the battery
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        battery_.name, "voltage", &battery_.voltage));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        battery_.name, "percentage", &battery_.percentage));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        battery_.name, "current", &battery_.current));


    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> CurioBotSystemHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.cmd));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.cmd));

    return command_interfaces;
}

CallbackReturn CurioBotSystemHardware::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("CurioBotSystemHardware"), "Activating ...please wait...");
    comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);
    RCLCPP_INFO(rclcpp::get_logger("CurioBotSystemHardware"), "Successfully activated!");

    return CallbackReturn::SUCCESS;
}

CallbackReturn CurioBotSystemHardware::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("CurioBotSystemHardware"), "Deactivating ...please wait...");
    comms_.disconnect();
    RCLCPP_INFO(rclcpp::get_logger("CurioBotSystemHardware"), "Successfully deactivated!");

    return CallbackReturn::SUCCESS;
}   

hardware_interface::return_type CurioBotSystemHardware::read()
{
    // RCLCPP_INFO(rclcpp::get_logger("CurioBotSystemHardware"), "Reading...");
    comms_.read_encoder_values(wheel_l_.enc, wheel_r_.enc);

    double pos_prev = wheel_l_.pos; 
    wheel_l_.pos = wheel_l_.calc_enc_angle();
    wheel_l_.vel = (wheel_l_.pos - pos_prev);

    pos_prev = wheel_r_.pos;
    wheel_r_.pos = wheel_r_.calc_enc_angle();
    wheel_r_.vel = (wheel_r_.pos - pos_prev);

    // Read data from TOF sensor
    comms_.read_sensor_values(sensor_n_.reading, sensor_l_.reading, sensor_r_.reading);
    comms_.read_up_sensor_values(sensor_n1_.reading, sensor_l1_.reading, sensor_r1_.reading);

    //Read Volstge Data from Battery
    double battery_voltage;
    comms_.read_battery_volt(battery_voltage);
    battery_.update_values(battery_voltage);

    // Publish sensor data
    // sensor_msgs::msg::Range range_msg;
    // range_msg.header.stamp = node_->now();
    // range_msg.header.frame_id = "tof";
    // range_msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
    // range_msg.field_of_view = 0.52;
    // range_msg.min_range = 0.52;
    // range_msg.max_range = 4.0;
    // range_msg.range = sensor_n_.reading;
    // range_publisher_->publish(range_msg);

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type CurioBotSystemHardware::write()
{
    // RCLCPP_INFO(rclcpp::get_logger("CurioBotSystemHardware"), "Writing...");
    int motor_l_counts_per_loop = wheel_l_.cmd / wheel_l_.rads_per_count / cfg_.loop_rate;
    int motor_r_counts_per_loop = wheel_r_.cmd / wheel_r_.rads_per_count / cfg_.loop_rate;
    comms_.set_motor_values(motor_l_counts_per_loop, motor_r_counts_per_loop);

    return hardware_interface::return_type::OK;
}

}  // namespace curio_one

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    curio_one::CurioBotSystemHardware, hardware_interface::SystemInterface)
