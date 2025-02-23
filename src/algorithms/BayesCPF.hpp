/**
 * This file is copied from
 * https://github.com/khaiyichin/sensor-degradation-filter/blob/385c899db5e6aefa5b8802bef5657dda9819d91e/include/sensor_degradation_filter/algorithms/DynamicDegradationFilterDelta.hpp
 * with some modifications to strip out unnecessary dependencies for the Bayes CPF
 */

#ifndef BAYES_CPF_HPP
#define BAYES_CPF_HPP

#include "CollectivePerception.hpp"
#include "EKF.hpp"

#define SENSOR_ACCURACY_INTERNAL_UNIT_FACTOR_DELTA 1.0 // the sensor accuracy is magnified internally to prevent vanishing products

class BayesCPF : public EKF
{
public:
    enum class Variant
    {
        BinomialApproximation = 0,
    };

    struct BinomialApproximationParams
    {
        float FillRatioReference = -1.0;

        int ThresholdCount = 5;
    };

    struct TruncationParameters
    {
        FFPair StandardizedBounds = {0.0, 0.0};

        FFPair TransformedStateEstimate = {0.0, 0.0};

        float NormalizationFactor = 0.0;

        void StandardizeBounds(FFPair original_bounds, float mean, float std_dev);

        void ComputeNormalizationFactor();

        void ComputeTransformedStateEstimates();

        void Init();

        void Reset();
    };

    BayesCPF(const std::shared_ptr<CollectivePerception> &col_per_ptr,
                                  float lower_bound = 0.5 + ZERO_APPROX,
                                  float upper_bound = 1.0 - ZERO_APPROX);

    virtual void Init() override;

    virtual void Reset() override;

    virtual void Estimate();

private:
    virtual void Predict();

    virtual void Update();

    void ComputeConstrainedSensorAccuracy();

    std::shared_ptr<CollectivePerception> collective_perception_algo_ptr_;

    FFPair bounds_original_ = {-1.0, -1.0};

    FFPair bounds_internal_ = {-1.0, -1.0};

    TruncationParameters truncation_params_;

    BinomialApproximationParams bin_params_;

    Variant variant_;

    float prev_assumed_acc_ = -1.0;

    float internal_unit_factor_ = 1.0; // internal units to prevent vanishing products

    const std::vector<float> empty_vec_float_ = {}; // dummy variable; used to pass into the predict function which doesn't require additional arguments
};

#endif