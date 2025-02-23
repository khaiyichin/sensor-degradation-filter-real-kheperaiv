/**
 * This file is copied from
 * https://github.com/khaiyichin/sensor-degradation-filter/blob/385c899db5e6aefa5b8802bef5657dda9819d91e/src/algorithms/ExtendedKalmanFilter.cpp
 * with some modiciations to convert from double to float
 */

#include "EKF.hpp"

EKF::EKF() : SensorDegradationFilter() {}

void EKF::Reset()
{
    SensorDegradationFilter::Reset();

    kalman_gain_ = 0.0;

    prediction_ = {0.0, 0.0};

    update_ = initial_guess_;
}

FFPair EKF::Predict(const std::vector<float> &state_prediction_coefficients)
{
    prediction_.first = nonlinear_state_prediction_function_(update_.first, state_prediction_coefficients); // g(x, u)

    prediction_.second = std::pow(linearized_state_prediction_a_, 2 * params_ptr_->FilterActivationPeriodTicks) * update_.second + state_prediction_r_ * params_ptr_->FilterActivationPeriodTicks; // A^t * sigma^2 * (A^t)' + R*t

    return prediction_;
}

FFPair EKF::Update(float measurement, const std::vector<float> &measurement_update_coefficients)
{
    kalman_gain_ = prediction_.second * linearized_measurement_update_c_ /
                   (std::pow(linearized_measurement_update_c_, 2) * prediction_.second + measurement_update_q_); // pred_var * C' / (C * pred_var * C' + Q)

    update_.first = prediction_.first +
                    kalman_gain_ * (measurement - nonlinear_measurement_update_function_(prediction_.first, measurement_update_coefficients)); // pred_mean + K * (z - h(pred_mean))

    update_.second = (1 - kalman_gain_ * linearized_measurement_update_c_) * prediction_.second; // (I - K * C) * pred_var

    return update_;
}

void EKF::Estimate(float measurement, const std::vector<float> &state_prediction_coefficients, const std::vector<float> &measurement_update_coefficients)
{
    Predict(state_prediction_coefficients);

    Update(measurement, measurement_update_coefficients);
}