#include <iostream>
#include "SensorConfig.h"

ConfigSensor_t SensorConfig::configSensor;
std::queue<std::pair<unsigned int, std::variant<int32_t, float, bool> > >  SensorConfig::queueControls;

using namespace libcamera;

void SensorConfig::InitConfigs(std::shared_ptr<libcamera::Camera> camera) {
    std::cout << "\nControls:\n";
    for (auto cont: camera->controls()) {
        std::cout
                << "\tid: 0x" << std::hex << cont.first->id() << std::dec
                << "\tname: " << cont.first->name()
                << "\t " << cont.second.toString() << "\n";


        switch (cont.first->id()) {
            case controls::EXPOSURE_TIME: {
                configSensor.exposureTime = cont.second.def().get<int32_t>();
                break;
            }
            case controls::DIGITAL_GAIN: {
//                configSensor.digitalGain = cont.second.def().get<float>();
                break;
            }
            case controls::ANALOGUE_GAIN: {
//                configSensor.analogueGain = cont.second.def().get<float>();
                break;
            }
            case controls::SATURATION: {
//                configSensor.saturation = cont.second.def().get<float>();
                break;
            }
            case controls::SHARPNESS: {
//                configSensor.sharpness = cont.second.def().get<float>();
                break;
            }

        }
    }
}
