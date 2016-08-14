#include <scanner.h>
#include <pid.h>

#include <iostream>
#include <string>

int main() {
    scanner device("/dev/ttyUSB0",115200);
    std::cout << "Battery voltage (V):  "<< device.getVoltage() <<"\n";
    //Sample queries
    std::cout << "DTC Errors: " << device.dtcErrors() << "\n";
    if(device.dtcErrors() > 0)
        ;//device.displayDTC();

    //Engine queries
    std::cout << "Engine RPM (rpm): " << device.sendQuery(0x01,PID_RPM) << "\n";
    std::cout << "Calculate engine load(%): " << device.sendQuery(0x01,PID_ENGINE_LOAD) <<"\n";
    std::cout << "Engine coolant temperature (°C): "<< device.sendQuery(0x01,PID_COOLANT_TEMP) <<"\n";
    std::cout << "Calculated Engine load (%): " << device.sendQuery(0x01,PID_ENGINE_LOAD) << "\n";
    std::cout << "Absolute Engine load (%): " << device.sendQuery(0x01,PID_ABSOLUTE_ENGINE_LOAD) << "\n";
    std::cout << "Ignition timing advance (°): " << device.sendQuery(0x01,PID_TIMING_ADVANCE) << "\n";
    std::cout << "Engine oil temperature (°C): " << device.sendQuery(0x01,PID_ENGINE_OIL_TEMP) << "\n";
    std::cout << "Engine torque percentage (%): " << device.sendQuery(0x01,PID_ENGINE_TORQUE_PERCENTAGE) << "\n";
    std::cout << "Engine reference torque (Nm): " << device.sendQuery(0x01,PID_ENGINE_REF_TORQUE) << "\n";
    //Intake/Exhaust queries
    std::cout << "Intake temperature (°C): " << device.sendQuery(0x01,PID_INTAKE_TEMP) << "\n";
    std::cout << "Intake manifold absolute pressure (kPa): " << device.sendQuery(0x01,PID_INTAKE_MAP) << "\n";
    std::cout << "MAF flow pressure (grams/s): " << device.sendQuery(0x01,PID_MAF_FLOW) << "\n";
    std::cout << "Barometric pressure (kPa)" << device.sendQuery(0x01,PID_BAROMETRIC) << "\n";
    //Speed/Time queries
    std::cout << "Vehicle speed (km/h): " << device.sendQuery(0x01,PID_SPEED) << "\n";
    std::cout << "Engine running time (seconds): " << device.sendQuery(0x01, PID_RUNTIME) << "\n";
    std::cout << "Vehicle running distance (km): " << device.sendQuery(0x01,PID_DISTANCE) << "\n";
    //Driver queries
    std::cout << "Fuel level (%): " << device.sendQuery(0x01,PID_FUEL_LEVEL) << "\n";
    std::cout << "Throttle position (%): " << device.sendQuery(0x01,PID_THROTTLE) << "\n";
    std::cout << "Ambient temperature (ºC): " << device.sendQuery(0x01,PID_AMBIENT_TEMP) << "\n";
    //Electric Systems queries
    std::cout << "Vehicle control module voltage (V): " << device.sendQuery(0x01,PID_CONTROL_MODULE_VOLTAGE) << "\n";
    std::cout << "Hybrid battery pack remaining life (%): " << device.sendQuery(0x01,PID_HYBRID_BATTERY_PERCENTAGE) << "\n";
    std::cout << "Battery voltage (V):  "<< device.getVoltage() <<"\n";

    return 0;
}
