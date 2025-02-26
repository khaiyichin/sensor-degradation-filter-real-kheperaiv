/**
 * This file is copied from
 * https://github.com/khaiyichin/sensor-degradation-filter/blob/385c899db5e6aefa5b8802bef5657dda9819d91e/src/controllers/BayesCPFDiffusionController.cpp
 * with some modifications to add WiFi communication, positioning sensor, and remote data logging functionality
 */

#include <algorithm>
#include <numeric>
#include <argos3/core/utility/logging/argos_log.h>
#include <thread>
#include <cstring>     // memset
#include <arpa/inet.h> // socket, connect, send
#include <unistd.h>    // close

#include "algorithms/BayesCPF.hpp"
#include "BayesCPFDiffusionController.hpp"

BayesCPFDiffusionController::WheelTurningParams::WheelTurningParams() : TurnMech(TurningMechanism::NO_TURN),
                                                                        HardTurnOnAngleThreshold(ToRadians(CDegrees(90.0))),
                                                                        SoftTurnOnAngleThreshold(ToRadians(CDegrees(70.0))),
                                                                        NoTurnAngleThreshold(ToRadians(CDegrees(10.0))),
                                                                        MaxSpeed(10.0)
{
}

void BayesCPFDiffusionController::WheelTurningParams::Init(TConfigurationNode &xml_node)
{
    try
    {
        TurnMech = TurningMechanism::NO_TURN;
        CDegrees angle_deg;
        GetNodeAttribute(xml_node, "hard_turn_angle_threshold", angle_deg);
        HardTurnOnAngleThreshold = ToRadians(angle_deg);
        GetNodeAttribute(xml_node, "soft_turn_angle_threshold", angle_deg);
        SoftTurnOnAngleThreshold = ToRadians(angle_deg);
        GetNodeAttribute(xml_node, "no_turn_angle_threshold", angle_deg);
        NoTurnAngleThreshold = ToRadians(angle_deg);
        GetNodeAttribute(xml_node, "max_speed", MaxSpeed);
    }
    catch (CARGoSException &ex)
    {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing controller wheel turning parameters.", ex);
    }
}

/****************************************/
/****************************************/

BayesCPFDiffusionController::DiffusionParams::DiffusionParams() : GoStraightAngleRange(CRadians(-1.0f), CRadians(1.0f)) {}

void BayesCPFDiffusionController::DiffusionParams::Init(TConfigurationNode &xml_node)
{
    try
    {
        CRange<CDegrees> cGoStraightAngleRangeDegrees(CDegrees(-10.0f), CDegrees(10.0f));
        GetNodeAttribute(xml_node, "go_straight_angle_range", cGoStraightAngleRangeDegrees);
        GoStraightAngleRange.Set(ToRadians(cGoStraightAngleRangeDegrees.GetMin()),
                                 ToRadians(cGoStraightAngleRangeDegrees.GetMax()));
        GetNodeAttribute(xml_node, "delta", Delta);
        GetNodeAttribute(xml_node, "bounds_x", BoundsX);
        GetNodeAttribute(xml_node, "bounds_y", BoundsY);
        // DEBUG
        THROW_ARGOSEXCEPTION("bounds not implemented yet!!");
    }
    catch (CARGoSException &ex)
    {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing controller diffusion parameters.", ex);
    }
}

/****************************************/
/****************************************/

BayesCPFDiffusionController::BayesCPFDiffusionController() : ci_wheels_ptr_(NULL),
                                                             ci_positioning_sensor_ptr_(NULL),
                                                             ci_wifi_actuator_ptr_(NULL),
                                                             ci_wifi_sensor_ptr_(NULL),
                                                             ci_ground_ptr_(NULL),
                                                             ci_proximity_ptr_(NULL),
                                                             sensor_degradation_filter_ptr_(NULL),
                                                             collective_perception_algo_ptr_(std::make_shared<CollectivePerception>())
{
}

void BayesCPFDiffusionController::Init(TConfigurationNode &xml_node)
{
    try
    {
        /* Get pointers to devices */
        try
        {
            ci_wheels_ptr_ = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
            wheel_turning_params_.Init(GetNode(xml_node, "wheel_turning"));
            // ci_positioning_sensor_ptr_ = GetSensor<CCI_PositioningSensor>("kheperaiv_positioning_vicon");
            ci_wifi_sensor_ptr_ = GetSensor<CCI_KheperaIVWiFiSensor>("kheperaiv_wifi");
            ci_wifi_actuator_ptr_ = GetActuator<CCI_KheperaIVWiFiActuator>("kheperaiv_wifi");
            ci_ground_ptr_ = GetSensor<CCI_KheperaIVGroundSensor>("kheperaiv_ground");
            ci_proximity_ptr_ = GetSensor<CCI_KheperaIVProximitySensor>("kheperaiv_proximity");

            // comms_params_.RABDataSize = ci_rab_actuator_ptr_->GetSize();
        }
        catch (CARGoSException &ex)
        {
            THROW_ARGOSEXCEPTION_NESTED("Error initializing the KheperaIV diffusion motion controller for robot \"" << GetId() << "\"", ex);
        }
    }
    catch (CARGoSException &ex)
    {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing the Buzz controller for the Khepera IV", ex);
    }

    /*
     * Parse XML parameters
     */
    /* Diffusion algorithm */
    diffusion_params_.Init(GetNode(xml_node, "diffusion"));
    /* Wheel turning */
    wheel_turning_params_.Init(GetNode(xml_node, "wheel_turning"));

    // Populate robot parameters
    GetNodeAttribute(GetNode(xml_node, "ground_sensor"), "period_ticks", ground_sensor_params_.GroundMeasurementPeriodTicks);
    GetNodeAttribute(GetNode(xml_node, "ground_sensor"), "sensor_acc_b", ground_sensor_params_.ActualSensorAcc["b"]);
    GetNodeAttribute(GetNode(xml_node, "ground_sensor"), "sensor_acc_w", ground_sensor_params_.ActualSensorAcc["w"]);
    GetNodeAttribute(GetNode(xml_node, "ground_sensor"), "sim", ground_sensor_params_.IsSimulated);
    GetNodeAttribute(GetNode(xml_node, "ground_sensor"), "dynamic", ground_sensor_params_.IsDynamic);
    GetNodeAttribute(GetNode(xml_node, "ground_sensor"), "true_deg_drift_coeff", ground_sensor_params_.DegradationCoefficients["drift"]);
    GetNodeAttribute(GetNode(xml_node, "ground_sensor"), "true_deg_diffusion_coeff", ground_sensor_params_.DegradationCoefficients["diffusion"]);
    GetNodeAttributeOrDefault(GetNode(xml_node, "ground_sensor"), "lowest_degraded_acc_lvl", ground_sensor_params_.LowestDegradedAccuracyLevel, float(0.5 + ZERO_APPROX));
    GetNodeAttribute(GetNode(xml_node, "comms"), "period_ticks", comms_params_.CommsPeriodTicks);
    GetNodeAttribute(GetNode(xml_node, "comms"), "single_hop_radius", comms_params_.SingleHopRadius);

    ground_sensor_params_.InitialActualAcc = ground_sensor_params_.ActualSensorAcc; // keep a copy of the original

    if (ground_sensor_params_.LowestDegradedAccuracyLevel == 0.5)
    {
        ground_sensor_params_.LowestDegradedAccuracyLevel = 0.5 + ZERO_APPROX; // to prevent numerical issues
    }
    else if (ground_sensor_params_.LowestDegradedAccuracyLevel < 0.5)
    {
        THROW_ARGOSEXCEPTION("Cannot have a sensor accuracy lower than 0.5.");
    }

    // Get ARGoS server parameters
    GetNodeAttribute(GetNode(xml_node, "argos_server"), "address", argos_server_params_.Address);
    GetNodeAttribute(GetNode(xml_node, "argos_server"), "port", argos_server_params_.Port);

    // Initialize Bayes CPF
    TConfigurationNode &sensor_degradation_filter_node = GetNode(xml_node, "sensor_degradation_filter");

    std::string method;

    GetNodeAttribute(sensor_degradation_filter_node, "method", method);

    std::transform(method.begin(),
                   method.end(),
                   method.begin(),
                   ::toupper);

    std::string pred_deg_model_B_str, pred_deg_var_R_str, init_mean_str, init_var_str, variant_str;
    float lowest_assumed_acc_lvl;

    // Check to see if the lowest assumed accuracy level possible is provided
    GetNodeAttributeOrDefault(GetNode(sensor_degradation_filter_node, "params"), "lowest_assumed_acc_lvl", lowest_assumed_acc_lvl, float(0.5 + ZERO_APPROX));

    if (lowest_assumed_acc_lvl == 0.5)
    {
        lowest_assumed_acc_lvl = 0.5 + ZERO_APPROX; // to prevent numerical issues
    }
    else if (lowest_assumed_acc_lvl < 0.5)
    {
        THROW_ARGOSEXCEPTION("Cannot have an assumed sensor accuracy lower than 0.5.");
    }

    sensor_degradation_filter_ptr_ =
        std::make_shared<BayesCPF>(collective_perception_algo_ptr_, lowest_assumed_acc_lvl);
    sensor_degradation_filter_ptr_->GetParamsPtr()->Method = "DELTA";

    GetNodeAttribute(GetNode(sensor_degradation_filter_node, "params"), "pred_deg_model_B", pred_deg_model_B_str);
    GetNodeAttribute(GetNode(sensor_degradation_filter_node, "params"), "pred_deg_var_R", pred_deg_var_R_str);
    GetNodeAttribute(GetNode(sensor_degradation_filter_node, "params"), "init_mean", init_mean_str);
    GetNodeAttribute(GetNode(sensor_degradation_filter_node, "params"), "init_var", init_var_str);
    GetNodeAttribute(GetNode(sensor_degradation_filter_node, "params"), "variant", variant_str);

    sensor_degradation_filter_ptr_->GetParamsPtr()->FilterSpecificParams = {{"pred_deg_model_B", pred_deg_model_B_str},
                                                                            {"pred_deg_var_R", pred_deg_var_R_str},
                                                                            {"init_mean", init_mean_str},
                                                                            {"init_var", init_var_str},
                                                                            {"variant", variant_str},
                                                                            {"lowest_assumed_acc_lvl", std::to_string(lowest_assumed_acc_lvl)}};

    SensorDegradationFilter::Params &sensor_degradation_filter_params = *sensor_degradation_filter_ptr_->GetParamsPtr();

    // Set the assumed accuracies to be the same for now; they will be updated by the loop functions if needed
    /*
        In the simulated case, the proper assumed accuracy for the robot is set by the loop functions;
        here we just initialized it to be the same as the actual accuracy but expect it to be changed before
        the experiment starts.

        In the non-simulated case, i.e., tracked ground sensor, the actual accuracy ground truth is not known and not
        required anyway, so the values obtained from the XML file for `sensor_acc_*` is used to parametrize our
        assumed accuracies. This works out because we don't simulate the tile color measurements in physical
        experiments but instead use the actual readings from the ground sensor. In other words, the values from
        `sensor_acc_*` from the XML file is our 'flawed' assumption of what the actual accuracy is.

        NOTE: in the non-simulated case, there should be no flawed robots, so the `num` attribute in the `flawed_robots`
        node (within the `sensor_degradation` loop functions node) should be 0. Consequently, the `acc_b` and `acc_w`
        attributes do not apply (though you should set it to be equal to the `sensor_acc_*` values in the controller
        node to facilitate data processing).
    */
    sensor_degradation_filter_params.AssumedSensorAcc["b"] = ground_sensor_params_.ActualSensorAcc["b"];
    sensor_degradation_filter_params.AssumedSensorAcc["w"] = ground_sensor_params_.ActualSensorAcc["w"];
    sensor_degradation_filter_params.RunDegradationFilter = false; // will be activated from the loop functions if required

    GetNodeAttribute(sensor_degradation_filter_node, "period_ticks", sensor_degradation_filter_params.FilterActivationPeriodTicks);

    // Check to see if the weighted average informed estimate is used for the fill ratio reference
    bool use_weighted_avg_informed_est;

    GetNodeAttributeOrDefault(sensor_degradation_filter_node, "use_weighted_avg_informed_est", use_weighted_avg_informed_est, false);

    // Check whether to use an observation queue
    UInt32 obs_queue_size;

    GetNodeAttribute(sensor_degradation_filter_node, "observation_queue_size", obs_queue_size);

    // Check to see if observation queue is desired
    if (obs_queue_size != 0)
    {
        collective_perception_algo_ptr_->GetParamsPtr()->UseObservationQueue = true;
        collective_perception_algo_ptr_->GetParamsPtr()->MaxObservationQueueSize = obs_queue_size;

        // Check to see if a dynamic-sized observation queue is desired
        GetNodeAttributeOrDefault(sensor_degradation_filter_node, "dynamic_observation_queue", sensor_degradation_filter_params.UseDynamicObservationQueue, false);

        if (sensor_degradation_filter_params.UseDynamicObservationQueue)
        {
            GetNodeAttribute(sensor_degradation_filter_node, "dynamic_observation_queue_window_size", sensor_degradation_filter_params.DynamicObservationQueueWindowSize);
        }

        // Set the boolean for using weighted average informed estimates which is only possible because observation queue size > 0
        sensor_degradation_filter_params.UseWeightedAvgInformedEstimates = use_weighted_avg_informed_est;

        collective_perception_algo_ptr_->GetParamsPtr()->MaxInformedEstimateHistoryLength =
            sensor_degradation_filter_params.UseWeightedAvgInformedEstimates
                ? collective_perception_algo_ptr_->GetParamsPtr()->MaxObservationQueueSize
                : 1;
    }
    else
    {
        collective_perception_algo_ptr_->GetParamsPtr()->UseObservationQueue = false;

        // Warn that it's not possible to use the weighted average informed estimate because there isn't an observation queue
        if (use_weighted_avg_informed_est)
        {
            std::cout << "Cannot use weighted average informed estimate because observation queue isn't used." << std::endl;
            sensor_degradation_filter_params.UseWeightedAvgInformedEstimates = false;
        }
    }

    /* Create a random number generator. We use the 'argos' category so
       that creation, reset, seeding and cleanup are managed by ARGoS. */
    CRandom::CreateCategory("argos", 0); // need to create the category first for non-simulator controllers
    RNG_ptr_ = CRandom::CreateRNG("argos");

    // Initialize the filter algorithms
    sensor_degradation_filter_ptr_->Init();

    // Connect to the ARGoS server
    ConnectToARGoSServer();
}

void BayesCPFDiffusionController::ConnectToARGoSServer()
{
    // Create a socket
    socket_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (socket_ < 0)
    {
        THROW_ARGOSEXCEPTION("Error creating socket: " << ::strerror(errno));
    }

    // Configure server address
    sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = ::htons(argos_server_params_.Port);
    ::inet_pton(AF_INET, argos_server_params_.Address, &server_addr.sin_ addr); // convert string address to binary

    // Connect to server
    if (::connect(socket_, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
    {
        THROW_ARGOSEXCEPTION("Error connecting to socket server: " << strerror(errno));
    }

    LOG << "Connected to server!" << std::endl;

    // Create thread to listen to server
    std::thread listener_thread(&BayesCPFDiffusionController::ListenToServer, this);
    listener_thread.detach(); // Ensure thread completes before exiting
}

void BayesCPFDiffusionController::Reset()
{
    THROW_ARGOSEXCEPTION("Reset functionality is not available for real robot controller.");
}

std::vector<Real> BayesCPFDiffusionController::GetData() const
{
    return {static_cast<Real>(RNG_ptr_->GetSeed()),
            static_cast<Real>(collective_perception_algo_ptr_->GetParamsPtr()->NumBlackTilesSeen),
            static_cast<Real>(collective_perception_algo_ptr_->GetParamsPtr()->NumObservations),
            collective_perception_algo_ptr_->GetLocalVals().X,
            collective_perception_algo_ptr_->GetLocalVals().Confidence,
            collective_perception_algo_ptr_->GetSocialVals().X,
            collective_perception_algo_ptr_->GetSocialVals().Confidence,
            collective_perception_algo_ptr_->GetInformedVals().X,
            sensor_degradation_filter_ptr_->GetParamsPtr()->AssumedSensorAcc.at("b"),
            sensor_degradation_filter_ptr_->GetParamsPtr()->AssumedSensorAcc.at("w"),
            ground_sensor_params_.ActualSensorAcc.at("b"),
            ground_sensor_params_.ActualSensorAcc.at("w"),
            collective_perception_algo_ptr_->GetParamsPtr()->WeightedAverageInformedEstimate};
}

void BayesCPFDiffusionController::ControlStep()
{
    if (start_flag_.load(std::memory_order_acquire))
    {
        ++tick_counter_;

        // Move robot
        SetWheelSpeedsFromVector(ComputeDiffusionVector());

        // Update information on self position
        GetSelfPosition();

        // Collect ground measurement and compute local estimate
        if (tick_counter_ % ground_sensor_params_.GroundMeasurementPeriodTicks == 0)
        {
            // Check if an observation queue is used
            if (!collective_perception_algo_ptr_->GetParamsPtr()->UseObservationQueue)
            {
                collective_perception_algo_ptr_->GetParamsPtr()->NumBlackTilesSeen += 1 - ObserveTileColor(); // the collective perception algorithm flips the black and white tiles
                ++collective_perception_algo_ptr_->GetParamsPtr()->NumObservations;
            }
            else // use an observation queue
            {
                size_t dynamic_queue_size = collective_perception_algo_ptr_->GetParamsPtr()->MaxObservationQueueSize;

                // Check to see if dynamic queue size is used
                if (sensor_degradation_filter_ptr_->GetParamsPtr()->UseDynamicObservationQueue &&
                    tick_counter_ >= sensor_degradation_filter_ptr_->GetParamsPtr()->DynamicObservationQueueWindowSize * ground_sensor_params_.GroundMeasurementPeriodTicks)
                {
                    // Check whether it's the first time we're calculating the dynamic queue size
                    if (tick_counter_ == sensor_degradation_filter_ptr_->GetParamsPtr()->DynamicObservationQueueWindowSize * ground_sensor_params_.GroundMeasurementPeriodTicks)
                    {
                        // Store the assumed accuracy for the very first time (so that we actually have the accuracy from one time step before in the later time steps)
                        prev_assumed_acc_ = sensor_degradation_filter_ptr_->GetParamsPtr()->AssumedSensorAcc["b"]; // update with the latest assumed accuracy
                    }
                    else
                    {
                        // Collect the current degradation rate and fill ratio reference (i.e., the informed estimate)
                        previous_degradation_rates_and_fill_ratio_references_.push_back(std::pair<float, float>((sensor_degradation_filter_ptr_->GetParamsPtr()->AssumedSensorAcc["b"] - prev_assumed_acc_) / ground_sensor_params_.GroundMeasurementPeriodTicks,
                                                                                                                collective_perception_algo_ptr_->GetInformedVals().X));

                        // Maintain the fixed window size
                        if (previous_degradation_rates_and_fill_ratio_references_.size() > sensor_degradation_filter_ptr_->GetParamsPtr()->DynamicObservationQueueWindowSize)
                        {
                            previous_degradation_rates_and_fill_ratio_references_.pop_front(); // ensure the queue stays true to the desired window size
                        }
                        prev_assumed_acc_ = sensor_degradation_filter_ptr_->GetParamsPtr()->AssumedSensorAcc["b"]; // update with the latest assumed accuracy

                        // Compute the average values
                        averaged_deg_rates_and_fill_ratio_refs_ = std::accumulate(previous_degradation_rates_and_fill_ratio_references_.begin(),
                                                                                  previous_degradation_rates_and_fill_ratio_references_.end(),
                                                                                  std::pair<Real, Real>(0.0, 0.0),
                                                                                  [](std::pair<Real, Real> left, std::pair<Real, Real> right)
                                                                                  {
                                                                                      return std::pair<Real, Real>(left.first + right.first, left.second + right.second);
                                                                                  });

                        averaged_deg_rates_and_fill_ratio_refs_ = {averaged_deg_rates_and_fill_ratio_refs_.first / previous_degradation_rates_and_fill_ratio_references_.size(),
                                                                   sensor_degradation_filter_ptr_->GetParamsPtr()->UseWeightedAvgInformedEstimates
                                                                       ? collective_perception_algo_ptr_->GetParamsPtr()->WeightedAverageInformedEstimate
                                                                       : averaged_deg_rates_and_fill_ratio_refs_.second / previous_degradation_rates_and_fill_ratio_references_.size()};

                        // Calculate dynamic queue size from the averaged values
                        if (std::abs(averaged_deg_rates_and_fill_ratio_refs_.first) < ZERO_APPROX)
                        {
                            dynamic_queue_size = collective_perception_algo_ptr_->GetParamsPtr()->MaxObservationQueueSize;
                        }
                        else
                        {
                            dynamic_queue_size = static_cast<int>(std::ceil(0.02 /
                                                                            std::abs(sensor_degradation_filter_ptr_->GetParamsPtr()->FilterActivationPeriodTicks *
                                                                                     (2.0 * averaged_deg_rates_and_fill_ratio_refs_.second - 1.0) *
                                                                                     averaged_deg_rates_and_fill_ratio_refs_.first)));
                        }
                    }
                    collective_perception_algo_ptr_->GetParamsPtr()->AddToQueue(1 - ObserveTileColor(), dynamic_queue_size); // add to observation queue
                }
                else // not using dynamic queue sizes
                {
                    collective_perception_algo_ptr_->GetParamsPtr()->AddToQueue(1 - ObserveTileColor(), -1);
                }
            }
            collective_perception_algo_ptr_->ComputeLocalEstimate(sensor_degradation_filter_ptr_->GetParamsPtr()->AssumedSensorAcc.at("b"),
                                                                  sensor_degradation_filter_ptr_->GetParamsPtr()->AssumedSensorAcc.at("w"));
        }

        // Communicate local estimates and compute social estimate
        if (tick_counter_ % comms_params_.CommsPeriodTicks == 0)
        {
            // Extract local estimate
            CollectivePerception::EstConfPair local_est = collective_perception_algo_ptr_->GetLocalVals();

            // Serialize data
            CByteArray data;

            {
                std::lock_guard<std::mutex> lock(self_pose_mutex_);

                data << GetId();
                data << self_pose_.GetX(); // TODO: obtained from the positioning_sensor
                data << self_pose_.GetY(); // TODO: obtained from the positioning_sensor
                data << local_est.X;
                data << local_est.Confidence;
            }

            // Broadcast data
            ci_wifi_actuator_ptr_->SendToMany(data);

            // Listen to neighbors, if any, and compute social estimate
            collective_perception_algo_ptr_->ComputeSocialEstimate(GetNeighborMessages());
        }

        // Compute informed estimate only if there are new local or social estimates
        if (tick_counter_ % ground_sensor_params_.GroundMeasurementPeriodTicks == 0 || tick_counter_ % comms_params_.CommsPeriodTicks == 0)
        {
            collective_perception_algo_ptr_->ComputeInformedEstimate();

            // Compute the weighted average informed estimates
            if (sensor_degradation_filter_ptr_->GetParamsPtr()->UseWeightedAvgInformedEstimates)
            {
                collective_perception_algo_ptr_->GetParamsPtr()->ComputeWeightedAverageFillRatioReference(collective_perception_algo_ptr_->GetInformedVals().X);
            }
        }

        // Run degradation filter
        if (sensor_degradation_filter_ptr_->GetParamsPtr()->RunDegradationFilter && tick_counter_ % sensor_degradation_filter_ptr_->GetParamsPtr()->FilterActivationPeriodTicks == 0)
        {
            sensor_degradation_filter_ptr_->Estimate();

            // Update sensor accuracies
            UpdateAssumedSensorAcc(sensor_degradation_filter_ptr_->GetAccuracyEstimates());
        }

        // Evolve sensor degradation
        if (ground_sensor_params_.IsSimulated && ground_sensor_params_.IsDynamic)
        {
            EvolveSensorDegradation();
        }

        // Send data for this timestep to the server
        SendDataToServer();
    }
}

std::vector<CollectivePerception::EstConfPair> BayesCPFDiffusionController::GetNeighborMessages()
{
    // Get all of the messages on the multicast address
    ci_wifi_sensor_ptr_->GetMessages(messages_vec_);

    std::vector<CollectivePerception::EstConfPair> neighbor_vals;

    if (messages_vec_.size() > 0)
    {
        CVector2 neighbor_position = CVector2::ZERO;

        Real distance;

        for (size_t i = 0; i < messages_vec_.size(); ++i)
        {
            std::string id_str;
            Real local_est, local_conf, neighbor_x, neighbor_y;

            messages_vec_[i].Payload >> id_str;
            messages_vec_[i].Payload >> neighbor_x;
            messages_vec_[i].Payload >> neighbor_y;
            messages_vec_[i].Payload >> local_est;
            messages_vec_[i].Payload >> local_conf;

            neighbor_position.Set(neighbor_x, neighbor_y);

            // Check if distance is too far for robot to be considered a neighbor
            distance = Distance(self_pose_, neighbor_position);

            if (distance > comms_params_.SingleHopRadius)
            {
                continue;
            }
            else
            {
                CollectivePerception::EstConfPair neighbor_val;

                neighbor_val.Id = id_str;
                neighbor_val.X = local_est;
                neighbor_val.Confidence = local_conf;

                neighbor_vals.push_back(neighbor_val);
            }
        }

        // Clean up
        ci_wifi_sensor_ptr_->FlushMessages();
    }

    return neighbor_vals;
}

void BayesCPFDiffusionController::ListenToServer()
{
    UInt8 *buffer = new UInt8[server_msg_size_]; // 64 bytes should be sufficient
    ssize_t bytes_received;
    CByteArray received_data;

    // Format: start_bit (UInt8), ID (string), X (Real), Y (Real), Theta (Real)
    UInt8 start_bit = 0;
    std::string name;
    Real x, y, theta;

    while (!shutdown_flag_.load(std::memory_order_acquire))
    {
        // Receive message from server
        bytes_received = ::recv(socket_, buffer, server_msg_size_, 0);

        // Check if the connection is closed (0 bytes received)
        if (bytes_received == 0)
        {
            THROW_ARGOSEXCEPTION("Server disconnected.");
        }

        // Check for errors during receiving data
        if (bytes_received < 0)
        {
            THROW_ARGOSEXCEPTION("Error receiving data from server.");
        }

        // Store the received data
        received_data = CByteArray(buffer, bytes_received);
        received_data >> start_bit;
        received_data >> name;
        received_data >> x;
        received_data >> y;
        received_data >> theta;

        // Ensure the correct message has been received
        if (name != GetId())
        {
            THROW_ARGOSEXCEPTION("Received message was intended for " << name << ", not " << GetId());
        }

        // Set the position data
        {
            std::lock_guard<std::mutex> lock(self_pose_mutex_);
            self_pose_.SetX(x);
            self_pose_.SetY(y);
            self_pose_.SetZ(theta);
        }

        // Set the flag to start the robot operation
        if (start_bit == 1)
        {
            start_flag_.store(true, std::memory_order_release);
        }

        // Reduce CPU usage
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void BayesCPFDiffusionController::GetSelfPosition()
{
    // TODO: get position from positioning sensor
    // self_pose_ = ci_positioning_sensor_ptr_->GetReading().Position.ProjectOntoXY();

    // TODO: for now this is just set to the origin
    self_pose_ = CVector2::ZERO;
    {
        std::lock_guard<std::mutex> lock(self_pose_mutex_);
    }
}

void BayesCPFDiffusionController::SendDataToServer()
{
    CByteArray data_to_send;
    std::vector<Real> data = GetData();

    data_to_send << data.size();

    for (auto itr = data.begin(); itr != data.end(); ++itr)
    {
        data_to_send << *itr;
    }

    // Send the data
    if (::send(socket_, data.ToCArray(), data.Size(), 0) < 0)
    {
        THROW_ARGOSEXCEPTION("Error sending data to the ARGoS server:" << ::strerror(errno));
    }
}

void BayesCPFDiffusionController::EvolveSensorDegradation()
{
    // Simulate sensor degradation (assuming Wiener process)
    if (ground_sensor_params_.DegradationCoefficients["drift"] > 0.0)
    {
        THROW_ARGOSEXCEPTION("Can only simulate sensor degradation with a negative drift coefficient.");
    }

    ground_sensor_params_.ActualSensorAcc["b"] += RNG_ptr_->Gaussian(ground_sensor_params_.DegradationCoefficients["diffusion"], ground_sensor_params_.DegradationCoefficients["drift"]);

    // Saturate sensor accuracy levels
    if (ground_sensor_params_.ActualSensorAcc["b"] <= ground_sensor_params_.LowestDegradedAccuracyLevel)
    {
        ground_sensor_params_.ActualSensorAcc["b"] = ground_sensor_params_.LowestDegradedAccuracyLevel;
    }
    else if (ground_sensor_params_.ActualSensorAcc["b"] >= 1.0)
    {
        ground_sensor_params_.ActualSensorAcc["b"] = 1.0 - ZERO_APPROX;
    }

    ground_sensor_params_.ActualSensorAcc["w"] = ground_sensor_params_.ActualSensorAcc["b"];
}

UInt32 BayesCPFDiffusionController::ObserveTileColor()
{
    /*
     * The ground sensors are located on the bottom of the robot, and can
     * be used to perform line following.
     *
     * The readings are in the following order (seeing the robot from TOP,
     * battery socket is the BACK):
     *
     *      front
     *
     *      0   3    r
     * l             i
     * e  1       2  g
     * f             h
     * t             t
     *
     *       back
     */

    const CCI_KheperaIVGroundSensor::TReadings &ground_readings = ci_ground_ptr_->GetReadings();

    // Use only the right sensor (index 3) to observe
    unsigned int encounter = static_cast<unsigned int>(std::round(ground_readings[3].Value));

    // Check if the ground sensor readings are actual or simulated
    if (!ground_sensor_params_.IsSimulated)
    {
        return static_cast<UInt32>(encounter);
    }
    else
    {
        // Compute simulated encounter
        float prob;

        if (encounter == 1) // white tile
        {
            prob = ground_sensor_params_.ActualSensorAcc["w"];
        }
        else if (encounter == 0) // black tile
        {
            prob = ground_sensor_params_.ActualSensorAcc["b"];
        }
        else
        {
            THROW_ARGOSEXCEPTION("Invalid tile color observed.");
        }

        // Apply noise to observation
        if (RNG_ptr_->Uniform(standard_uniform_support_) < prob) // correct observation
        {
            return static_cast<UInt32>(encounter);
        }
        else // incorrect observation
        {
            return static_cast<UInt32>(1 - encounter);
        }
    }
}

CVector2 BayesCPFDiffusionController::ComputeDiffusionVector()
{
    /* Get readings from proximity sensor */
    const CCI_KheperaIVProximitySensor::TReadings &proximity_readings = ci_proximity_ptr_->GetReadings();

    /* Sum them together */
    CVector2 diffusion_vector;

    for (size_t i = 0; i < proximity_readings.size(); ++i)
    {
        diffusion_vector += CVector2(proximity_readings[i].Value, proximity_readings[i].Angle);
    }
    /* If the angle of the vector is small enough and the closest obstacle
       is far enough, ignore the vector and go straight, otherwise return
       it */
    if (diffusion_params_.GoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(diffusion_vector.Angle()) &&
        diffusion_vector.Length() < diffusion_params_.Delta)
    {
        return CVector2::X * wheel_turning_params_.MaxSpeed;
    }
    else
    {
        diffusion_vector.Normalize();
        return -diffusion_vector * wheel_turning_params_.MaxSpeed;
    }
}

void BayesCPFDiffusionController::SetWheelSpeedsFromVector(const CVector2 &heading_vector)
{
    /* Get the heading angle */
    CRadians heading_angle_rad = heading_vector.Angle().SignedNormalize();

    /* Get the length of the heading vector */
    Real heading_length = heading_vector.Length();

    /* Clamp the speed so that it's not greater than MaxSpeed */
    Real base_wheel_spd = Min<Real>(heading_length, wheel_turning_params_.MaxSpeed);

    /* State transition logic */
    if (wheel_turning_params_.TurnMech == WheelTurningParams::TurningMechanism::HARD_TURN)
    {
        if (Abs(heading_angle_rad) <= wheel_turning_params_.SoftTurnOnAngleThreshold)
        {
            wheel_turning_params_.TurnMech = WheelTurningParams::TurningMechanism::SOFT_TURN;
        }
    }

    if (wheel_turning_params_.TurnMech == WheelTurningParams::TurningMechanism::SOFT_TURN)
    {
        if (Abs(heading_angle_rad) > wheel_turning_params_.HardTurnOnAngleThreshold)
        {
            wheel_turning_params_.TurnMech = WheelTurningParams::TurningMechanism::HARD_TURN;
        }
        else if (Abs(heading_angle_rad) <= wheel_turning_params_.NoTurnAngleThreshold)
        {
            wheel_turning_params_.TurnMech = WheelTurningParams::TurningMechanism::NO_TURN;
        }
    }

    if (wheel_turning_params_.TurnMech == WheelTurningParams::TurningMechanism::NO_TURN)
    {
        if (Abs(heading_angle_rad) > wheel_turning_params_.HardTurnOnAngleThreshold)
        {
            wheel_turning_params_.TurnMech = WheelTurningParams::TurningMechanism::HARD_TURN;
        }
        else if (Abs(heading_angle_rad) > wheel_turning_params_.NoTurnAngleThreshold)
        {
            wheel_turning_params_.TurnMech = WheelTurningParams::TurningMechanism::SOFT_TURN;
        }
    }

    /* Wheel speeds based on current turning state */
    Real spd_1, spd_2;

    switch (wheel_turning_params_.TurnMech)
    {
    case WheelTurningParams::TurningMechanism::NO_TURN:
    {
        /* Just go straight */
        spd_1 = base_wheel_spd;
        spd_2 = base_wheel_spd;
        break;
    }
    case WheelTurningParams::TurningMechanism::SOFT_TURN:
    {
        /* Both wheels go straight, but one is faster than the other */
        Real fSpeedFactor = (wheel_turning_params_.HardTurnOnAngleThreshold - Abs(heading_angle_rad)) / wheel_turning_params_.HardTurnOnAngleThreshold;
        spd_1 = base_wheel_spd - base_wheel_spd * (1.0 - fSpeedFactor);
        spd_2 = base_wheel_spd + base_wheel_spd * (1.0 - fSpeedFactor);
        break;
    }
    case WheelTurningParams::TurningMechanism::HARD_TURN:
    {
        /* Opposite wheel speeds */
        spd_1 = -wheel_turning_params_.MaxSpeed;
        spd_2 = wheel_turning_params_.MaxSpeed;
        break;
    }
    }

    /* Apply the calculated speeds to the appropriate wheels */
    Real left_wheel_spd, right_wheel_spd;

    if (heading_angle_rad > CRadians::ZERO)
    {
        /* Turn Left */
        left_wheel_spd = spd_1;
        right_wheel_spd = spd_2;
    }
    else
    {
        /* Turn Right */
        left_wheel_spd = spd_2;
        right_wheel_spd = spd_1;
    }

    /* Finally, set the wheel speeds */
    left_wheel_spd = Min<Real>(left_wheel_spd, wheel_turning_params_.MaxSpeed);
    right_wheel_spd = Min<Real>(right_wheel_spd, wheel_turning_params_.MaxSpeed);

    ci_wheels_ptr_->SetLinearVelocity(left_wheel_spd, right_wheel_spd);
}

REGISTER_CONTROLLER(BayesCPFDiffusionController, "bayes_cpf_diffusion_controller")