/**
 * This file is copied from
 * https://github.com/khaiyichin/sensor-degradation-filter/blob/385c899db5e6aefa5b8802bef5657dda9819d91e/include/sensor_degradation_filter/algorithms/ExtendedKalmanFilter.hpp
 */

#ifndef EKF_HPP
#define EKF_HPP

#include <cmath>
#include <deque>
#include <functional>
#include <vector>

#include "SensorDegradationFilter.hpp"

using FFPair = std::pair<float, float>;

class EKF : public SensorDegradationFilter
{
public:
    EKF();

    virtual void Reset() override;

    virtual void Init() = 0; // force initialization so that the nonlinear functions can be populated

protected:
    virtual FFPair Predict(const std::vector<float> &state_prediction_coefficients);

    virtual FFPair Update(float measurement, const std::vector<float> &measurement_update_coefficients);

    virtual void Estimate(float measurement, const std::vector<float> &state_prediction_coefficients, const std::vector<float> &measurement_update_coefficients);

    FFPair initial_guess_ = {0.0, 0.0}; // initial guess to be used for the first prediction step; first: state estimate, second: state variance

    FFPair prediction_ = {0.0, 0.0}; // first: state estimate, second: state variance

    FFPair update_ = {0.0, 0.0}; // first: state estimate, second: state variance

    std::function<float(float, const std::vector<float> &)> nonlinear_state_prediction_function_;

    std::function<float(float, const std::vector<float> &)> nonlinear_measurement_update_function_;

    float exponential_smoothing_factor_ = 1.0; // ignore this for the time being

    float kalman_gain_ = 0.0; // Kalman gain

    float linearized_state_prediction_a_ = 0.0; // linearized coefficient for the state_prediction model (default value: zero)

    float linearized_state_prediction_b_ = 0.0; // linearized coefficient for the state prediction input (aka the drift coefficient) (default: no drift)

    float state_prediction_r_ = -1.0; // our model of what the diffusion coefficient is (default: invalid value; requires setting)

    float linearized_measurement_update_c_ = 0.0; // linearized coefficient for the measurement model (default: not observing; requires setting)

    float measurement_update_q_ = -1.0; // our model of what the measurement noise model is (default: invalid value; requires setting)
};

#endif