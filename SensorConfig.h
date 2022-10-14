//
// Created by tymbys on 12.06.2021.
//

#ifndef ASTROCAMERA_SENSORCONFIG_H
#define ASTROCAMERA_SENSORCONFIG_H

#include <queue>
#include <variant>
#include <cinttypes>
#include <libcamera/camera.h>
#include <libcamera/control_ids.h>

struct ConfigSensor_t {
    int32_t exposureTime;
    float analogueGain;
    float digitalGain;
    float saturation;
    float sharpness;

    friend bool operator==(const ConfigSensor_t & l, const ConfigSensor_t & r) {
        return (l.digitalGain == r.digitalGain) &&
        (l.analogueGain == r.analogueGain) &&
        (l.exposureTime == r.exposureTime) &&
        (l.sharpness == r.sharpness) &&
        (l.saturation == r.saturation);
    }

    friend bool operator!=(const ConfigSensor_t & l, const ConfigSensor_t & r) {
        return (l.digitalGain != r.digitalGain) ||
               (l.analogueGain != r.analogueGain) ||
               (l.exposureTime != r.exposureTime) ||
               (l.sharpness != r.sharpness) ||
               (l.saturation != r.saturation);
    }
};


typedef  std::variant<int32_t, float, bool> ValueConf;

class SensorConfig {
public:



    static void InitConfigs(std::shared_ptr<libcamera::Camera> camera);
    static ConfigSensor_t configSensor;
    static std::queue<std::pair<unsigned int, ValueConf > > queueControls;

};





#endif //ASTROCAMERA_SENSORCONFIG_H
