#pragma once

#include <string>

class HydraulicSystemMonitor {
private:
    std::string hydraulic_oil_temperature;
    std::string hydraulic_oil_tank_fill_level;
    std::string hydraulic_oil_pressure;

public:
    // setters
    void setOilTemperature(const std::string& temperature);
    void setOilTankFillLevel(const std::string& fill_level);
    void setOilPressure(const std::string& pressure);
    
    // getters
    std::string getOilTemperature() const;
    std::string getOilTankFillLevel() const;
    std::string getOilPressure() const;
};
