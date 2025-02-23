/**
 * This file is copied from
 * https://github.com/khaiyichin/sensor-degradation-filter/blob/385c899db5e6aefa5b8802bef5657dda9819d91e/src/algorithms/CollectivePerception.cpp
 * with some modifications to convert from float to float
 */

#include "CollectivePerception.hpp"

void CollectivePerception::ComputeLocalEstimate(const float &sensor_acc_b, const float &sensor_acc_w)
{
    float h = static_cast<float>(params_ptr_->NumBlackTilesSeen);
    float t = static_cast<float>(params_ptr_->NumObservations);

    float regular_estimate = (h / t + sensor_acc_w - 1.0) / (sensor_acc_b + sensor_acc_w - 1);

    if ((sensor_acc_b == 1.0) && (sensor_acc_w == 1.0)) // perfect sensor
    {
        local_vals_.X = h / t;
        local_vals_.Confidence = std::pow(t, 3) / (h * (t - h));
    }
    else // imperfect sensor
    {
        if (regular_estimate <= 0.0)
        {
            float num = std::pow(sensor_acc_b + sensor_acc_w - 1.0, 2) * (t * std::pow(sensor_acc_w, 2) - 2 * (t - h) * sensor_acc_w + (t - h));
            float denom = std::pow(sensor_acc_w, 2) * std::pow(sensor_acc_w - 1.0, 2);

            local_vals_.X = 0.0;
            local_vals_.Confidence = num / denom;
        }
        else if (regular_estimate >= 1.0)
        {
            float num = std::pow(sensor_acc_b + sensor_acc_w - 1.0, 2) * (t * std::pow(sensor_acc_b, 2) - 2 * h * sensor_acc_b + h);
            float denom = std::pow(sensor_acc_b, 2) * std::pow(sensor_acc_b - 1.0, 2);

            local_vals_.X = 1.0;
            local_vals_.Confidence = num / denom;
        }
        else
        {
            local_vals_.X = regular_estimate;
            local_vals_.Confidence = (std::pow(t, 3) * std::pow(sensor_acc_b + sensor_acc_w - 1.0, 2)) / (h * (t - h));
        }
    }

    // Modify confidence values to be non-zero to prevent numerical errors
    if (local_vals_.Confidence <= ZERO_APPROX)
    {
        local_vals_.Confidence = ZERO_APPROX;
    }
}

void CollectivePerception::ComputeSocialEstimate(const std::vector<EstConfPair> &neighbor_vals)
{
    EstConfPair sum;

    params_ptr_->MostRecentNeighborEstimates = neighbor_vals;

    auto lambda = [](const EstConfPair &left, const EstConfPair &right) -> EstConfPair
    {
        return EstConfPair(left.X + right.X * right.Confidence, left.Confidence + right.Confidence);
    };

    sum = std::accumulate(neighbor_vals.begin(), neighbor_vals.end(), EstConfPair(0.0, ZERO_APPROX), lambda);

    // Assign the averages as social values
    social_vals_.X = sum.X / sum.Confidence;
    social_vals_.Confidence = neighbor_vals.empty() ? 0.0 : sum.Confidence; // ensure no dummy values contribute to the informed estimates
}

void CollectivePerception::ComputeInformedEstimate()
{
    informed_vals_.X = (local_vals_.Confidence * local_vals_.X + social_vals_.Confidence * social_vals_.X) / (local_vals_.Confidence + social_vals_.Confidence);
    informed_vals_.Confidence = local_vals_.Confidence + social_vals_.Confidence;
}