/**
 * This file is copied from
 * https://github.com/khaiyichin/sensor-degradation-filter/blob/385c899db5e6aefa5b8802bef5657dda9819d91e/include/sensor_degradation_filter/algorithms/SensorDegradationFilter.hpp
 * with some modifications to convert from double to float
 */
#ifndef SENSOR_DEGRADATION_FILTER_HPP
#define SENSOR_DEGRADATION_FILTER_HPP

#include <unordered_map>
#include <memory>
#include <string>

class SensorDegradationFilter
{
public:
    struct Params
    {
        void Reset()
        {
            AssumedSensorAcc = InitialAssumedAcc;
        }

        std::string Method;

        bool RunDegradationFilter = false;

        bool UseWeightedAvgInformedEstimates = false;

        unsigned int FilterActivationPeriodTicks = 0;

        bool UseDynamicObservationQueue = false;

        int DynamicObservationQueueWindowSize = -1;

        std::unordered_map<std::string, std::string> FilterSpecificParams;

        std::unordered_map<std::string, float> AssumedSensorAcc = {{"b", -1.0}, {"w", -1.0}};

        std::unordered_map<std::string, float> InitialAssumedAcc = {{"b", -1.0}, {"w", -1.0}};
    };

    SensorDegradationFilter() : params_ptr_(std::make_shared<Params>()) {}

    virtual ~SensorDegradationFilter() {}

    virtual void Init() {}

    virtual void Reset()
    {
        params_ptr_->Reset();
    }

    virtual void Estimate() = 0;

    std::shared_ptr<Params> GetParamsPtr() const { return params_ptr_; }

    std::unordered_map<std::string, float> GetAccuracyEstimates() const { return params_ptr_->AssumedSensorAcc; };

protected:
    std::shared_ptr<Params> params_ptr_;
};

#endif