#ifndef CURIO_ONE_BATTERY_HPP
#define CURIO_ONE_BATTERY_HPP

#include <string>
#include <cmath>

class Battery
{
public:
    std::string name = "";
    double voltage = 0.0;
    double percentage = 0.0;
    double current = 0.0;

    Battery() = default;

    Battery(const std::string &battery_name)
    {
        setup(battery_name);
    }

    void setup(const std::string &battery_name)
    {
        name = battery_name;
    }

    void update_values(double voltage_reading)
    {
        voltage = voltage_reading;
        percentage = calc_percentage(voltage);
        current = calc_current(voltage);
    }

private:
    double calc_percentage(double voltage)
    {
        // Implement the logic to calculate battery percentage based on voltage
        // This is an example, adjust based on your battery specs
        double min_voltage = 9.0;  // Minimum voltage for the battery
        double max_voltage = 12.6;  // Maximum voltage for the battery
        return ((voltage - min_voltage) / (max_voltage - min_voltage)) * 100.0;
    }

    double calc_current(double voltage)
    {
        // Implement the logic to calculate current based on voltage
        // This is a placeholder example
        return voltage / 10.0;  // Adjust based on your system
    }
};

#endif // CURIO_ONE_BATTERY_HPP
