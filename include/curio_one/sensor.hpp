#ifndef CURIO_ONE_SENSOR_HPP
#define CURIO_ONE_SENSOR_HPP

#include <string>
#include <cmath>
#include "sensor_msgs/msg/range.hpp"


class Sensors
{
    public:

    std::string name = "";
    double reading = 0;

    Sensors() = default;

    Sensors(const std::string &sensor_name)
    {
      setup(sensor_name);
    }

    
    void setup(const std::string &sensor_name)
    {
      name = sensor_name;
    }

    double sensor_reading(sensor_msgs::msg::Range::SharedPtr sensor_reading) 
    {
      return reading;
    }


};


#endif // CURIO_ONE_SENSOR_HPP