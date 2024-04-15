#include "robot_info/hydraulic_system_monitor.h"

// setters
void HydraulicSystemMonitor::setOilTemperature(const std::string& temperature) {
    this->hydraulic_oil_temperature = temperature;
}
void HydraulicSystemMonitor::setOilTankFillLevel(const std::string& fill_level) {
    this->hydraulic_oil_tank_fill_level = fill_level;
}
void HydraulicSystemMonitor::setOilPressure(const std::string& pressure){
    this->hydraulic_oil_pressure = pressure;
}

// getters
std::string HydraulicSystemMonitor::getOilTemperature() const {
    return this->hydraulic_oil_temperature;
}
std::string HydraulicSystemMonitor::getOilTankFillLevel() const {
    return this->hydraulic_oil_tank_fill_level;
}
std::string HydraulicSystemMonitor::getOilPressure() const {
    return this->hydraulic_oil_pressure;
}
