/**
 * This file is copied from
 * https://github.com/khaiyichin/sensor-degradation-filter/blob/385c899db5e6aefa5b8802bef5657dda9819d91e/src/algorithms/DynamicDegradationFilterDelta.cpp
 * with some modifications to strip out unnecessary dependencies for the Bayes CPF
 */

#include "BayesCPF.hpp"
#include <iostream>
#include <limits>

void BayesCPF::TruncationParameters::Init()
{
    // nothing to be done
}

void BayesCPF::TruncationParameters::Reset()
{
    StandardizedBounds = {0.0, 0.0};
    TransformedStateEstimate = {0.0, 0.0};
    NormalizationFactor = 0.0;
}

void BayesCPF::TruncationParameters::StandardizeBounds(FFPair original_bounds, float mean, float std_dev)
{
    StandardizedBounds = {(original_bounds.first - mean) / std_dev,
                          (original_bounds.second - mean) / std_dev};
}

void BayesCPF::TruncationParameters::ComputeNormalizationFactor()
{
    // Calculate the denominator first
    NormalizationFactor = std::erf(StandardizedBounds.second / M_SQRT2) - std::erf(StandardizedBounds.first / M_SQRT2);

    // Prevent an infinite normalization factor (roundoff error); the usage of ZERO_APPROX squared is arbitrary since it represents a very small number
    if (NormalizationFactor < std::numeric_limits<float>::epsilon())
    {
        NormalizationFactor = std::sqrt(M_2_PI) / std::numeric_limits<float>::epsilon();
    }
    else
    {
        NormalizationFactor = std::sqrt(M_2_PI) / NormalizationFactor;
    }
}

void BayesCPF::TruncationParameters::ComputeTransformedStateEstimates()
{
    TransformedStateEstimate.first = NormalizationFactor *
                                     (std::exp(-std::pow(StandardizedBounds.first, 2) / 2) - std::exp(-std::pow(StandardizedBounds.second, 2) / 2));
    TransformedStateEstimate.second = NormalizationFactor *
                                          (std::exp(-std::pow(StandardizedBounds.first, 2) / 2) * (StandardizedBounds.first - 2 * TransformedStateEstimate.first) -
                                           std::exp(-std::pow(StandardizedBounds.second, 2) / 2) * (StandardizedBounds.second - 2 * TransformedStateEstimate.first)) +
                                      std::pow(TransformedStateEstimate.first, 2) + 1;
}

/******************************************************************/
/******************************************************************/

BayesCPF::BayesCPF(const std::shared_ptr<CollectivePerception> &col_per_ptr,
                   float lower_bound /* 0.5 + ZERO_APPROX */,
                   float upper_bound /* 1.0 - ZERO_APPROX */)
    : EKF(),
      bounds_original_({lower_bound, upper_bound}),
      bounds_internal_({lower_bound * SENSOR_ACCURACY_INTERNAL_UNIT_FACTOR_DELTA, upper_bound * SENSOR_ACCURACY_INTERNAL_UNIT_FACTOR_DELTA})
{
    collective_perception_algo_ptr_ = col_per_ptr;
}

void BayesCPF::Reset()
{
    // Call base Reset
    EKF::Reset();

    // Clear buffer for informed estimate (if used)
    collective_perception_algo_ptr_->GetParamsPtr()->InformedEstimateHistory.clear();

    // Reset parameters
    truncation_params_.Reset();
}

void BayesCPF::Init()
{
    // Initialize the internal unit factor
    internal_unit_factor_ = SENSOR_ACCURACY_INTERNAL_UNIT_FACTOR_DELTA;

    // Specify filter variant type
    if (params_ptr_->FilterSpecificParams["variant"] == "bin")
    {
        variant_ = Variant::BinomialApproximation;
    }
    else
    {
        throw std::invalid_argument("Unknown variant requested for the Delta filter; only \"bin\" is allowed.");
    }

    // Initialize common variables
    float init_mean, init_var;

    init_mean = std::stod(params_ptr_->FilterSpecificParams["init_mean"]) * internal_unit_factor_;
    init_var = std::stod(params_ptr_->FilterSpecificParams["init_var"]) * internal_unit_factor_ * internal_unit_factor_;

    initial_guess_ = {init_mean, init_var};

    // Initialize state prediction parameters (common across the variants)
    linearized_state_prediction_a_ = 1.0;
    linearized_state_prediction_b_ = std::stod(params_ptr_->FilterSpecificParams["pred_deg_model_B"].c_str()) * internal_unit_factor_;
    state_prediction_r_ = std::stod(params_ptr_->FilterSpecificParams["pred_deg_var_R"].c_str()) * internal_unit_factor_ * internal_unit_factor_;

    // Populate the nonlinear state prediction function of the EKF (it's actually linear)
    nonlinear_state_prediction_function_ = [this](float prev_mean, const std::vector<float> &input_coefficients)
    {
        return prev_mean +
               this->params_ptr_->FilterActivationPeriodTicks * this->linearized_state_prediction_b_;
    };

    // Initialize measurement update parameters
    linearized_measurement_update_c_ = collective_perception_algo_ptr_->GetParamsPtr()->NumObservations *
                                       (2 * bin_params_.FillRatioReference - 1) / internal_unit_factor_; // t * (2f - 1); needs to be updated every time step
    measurement_update_q_ = collective_perception_algo_ptr_->GetParamsPtr()->NumObservations *
                            (std::pow(2 * params_ptr_->AssumedSensorAcc["b"] / internal_unit_factor_ - 1, 2) * (bin_params_.FillRatioReference - std::pow(bin_params_.FillRatioReference, 2) - 0.25) +
                             0.25); // q = t * ( bf + (1-b)*(1-f)) * (1 - bf - (1-b)*(1-f)) )
                                    //   = t * ( (2*b - 1)^2 * (f - f^2 - 1/4) + 1/4 ); needs to be updated every time step

    // Initialize parameters based on the variants
    switch (variant_)
    {
    case Variant::BinomialApproximation:
    {
        // Populate the nonlinear measurement update function of the EKF (it's actually linear)
        nonlinear_measurement_update_function_ = [this](float pred_mean, const std::vector<float> &input_coefficients)
        {
            // n = t * ( bf + (1-b)*(1-f) )
            //   = t * ( f * (2*b - 1) - b + 1 )
            return this->collective_perception_algo_ptr_->GetParamsPtr()->NumObservations *
                   (this->bin_params_.FillRatioReference * (2 * pred_mean / this->internal_unit_factor_ - 1) - pred_mean / this->internal_unit_factor_ + 1);
        };

        break;
    }
    }

    // Initialize truncation parameters (used for the "post-processing" of the EKF outputs)
    truncation_params_.Init();

    // Populate initial guess
    update_ = initial_guess_;
}

void BayesCPF::Predict()
{
    // Run the base predict step
    EKF::Predict(empty_vec_float_);
}

void BayesCPF::Update()
{
    switch (variant_)
    {
    case Variant::BinomialApproximation:
    {
        bin_params_.FillRatioReference =
            params_ptr_->UseWeightedAvgInformedEstimates
                ? collective_perception_algo_ptr_->GetParamsPtr()->WeightedAverageInformedEstimate
                : collective_perception_algo_ptr_->GetInformedVals().X;

        // Use np >= 5 && nq >= 5 rule to see if normal approximation holds
        if (collective_perception_algo_ptr_->GetParamsPtr()->NumObservations * (prediction_.first / internal_unit_factor_) >= bin_params_.ThresholdCount &&
            collective_perception_algo_ptr_->GetParamsPtr()->NumObservations * (1 - prediction_.first / internal_unit_factor_) >= bin_params_.ThresholdCount)
        {
            // Update the measurement jacobian and noise models
            linearized_measurement_update_c_ = collective_perception_algo_ptr_->GetParamsPtr()->NumObservations *
                                               (2 * bin_params_.FillRatioReference - 1) / internal_unit_factor_;
            measurement_update_q_ = collective_perception_algo_ptr_->GetParamsPtr()->NumObservations *
                                    (std::pow(2 * prediction_.first / internal_unit_factor_ - 1, 2) * (bin_params_.FillRatioReference - std::pow(bin_params_.FillRatioReference, 2) - 0.25) +
                                     0.25);

            // Run the base update step
            EKF::Update(collective_perception_algo_ptr_->GetParamsPtr()->NumBlackTilesSeen, empty_vec_float_);
        }
        else // approximation doesn't hold up well, do nothing
        {
            update_ = prediction_;
        }
        break;
    }
    }
}

void BayesCPF::Estimate()
{
    // Execute prediction step
    Predict();

    if (std::isnan(prediction_.first) || std::isnan(prediction_.second))
    {
        throw std::runtime_error("(Variant = " + std::to_string(static_cast<int>(variant_)) + ") NaN values encountered in the predicted values.");
    }

    // Execute update step
    Update();

    if (std::isnan(update_.first) || std::isnan(update_.second))
    {
        throw std::runtime_error("(Variant = " + std::to_string(static_cast<int>(variant_)) + ") NaN values encountered in the updated values.");
    }

    // Find the constrained version of the sensor accuracy; the constrained values are not fed back into the filter
    ComputeConstrainedSensorAccuracy();

    // Update assumed sensor accuracy only if it's out of bounds (should be converted OUT of internal units)
    if (update_.first < bounds_original_.first || update_.first > bounds_original_.second)
    {
        params_ptr_->AssumedSensorAcc["b"] = (std::sqrt(update_.second) * truncation_params_.TransformedStateEstimate.first + update_.first) / internal_unit_factor_;

        // Prevent the assumed sensor accuracy from going outside of bounds (due to rounding errors)
        if (params_ptr_->AssumedSensorAcc["b"] > bounds_original_.second)
        {
            params_ptr_->AssumedSensorAcc["b"] = bounds_original_.second;
        }
        else if (params_ptr_->AssumedSensorAcc["b"] < bounds_original_.first)
        {
            params_ptr_->AssumedSensorAcc["b"] = bounds_original_.first;
        }
    }
    else
    {
        params_ptr_->AssumedSensorAcc["b"] = update_.first;
    }

    // Apply exponential smoothing (01/21/2025 note: this has been turned off for the time being, i.e., no smoothing applied.)
    if (prev_assumed_acc_ == -1.0)
    {
        prev_assumed_acc_ = params_ptr_->AssumedSensorAcc["b"];
    }
    else
    {
        params_ptr_->AssumedSensorAcc["b"] = exponential_smoothing_factor_ * params_ptr_->AssumedSensorAcc["b"] + (1 - exponential_smoothing_factor_) * prev_assumed_acc_;
        prev_assumed_acc_ = params_ptr_->AssumedSensorAcc["b"];
    }

    params_ptr_->AssumedSensorAcc["w"] = params_ptr_->AssumedSensorAcc["b"]; // black tile accuracy is equal to white tile accuracy
}

void BayesCPF::ComputeConstrainedSensorAccuracy()
{
    // Constrain the assumed sensor accuracy estimate according to
    // "Constrained Kalman filtering via density function truncation for turbofan engine health estimation" by Simon and Simon (2010)

    // Standardize the bounds
    truncation_params_.StandardizeBounds(bounds_internal_, update_.first, std::sqrt(update_.second));

    // Compute normalization factor
    truncation_params_.ComputeNormalizationFactor();

    // Compute transformed state estimates
    truncation_params_.ComputeTransformedStateEstimates();
}