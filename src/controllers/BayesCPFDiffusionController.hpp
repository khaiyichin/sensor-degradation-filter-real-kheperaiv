/**
 * This file is copied from
 * https://github.com/khaiyichin/sensor-degradation-filter/blob/385c899db5e6aefa5b8802bef5657dda9819d91e/include/sensor_degradation_filter/controllers/BayesCPFDiffusionController.hpp
 * with some modifications to add WiFi communication, positioning sensor, and remote data logging functionality
 */

#ifndef BAYES_CPF_DIFFUSION_CONTROLLER_HPP
#define BAYES_CPF_DIFFUSION_CONTROLLER_HPP

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <argos3/plugins/robots/kheperaiv/control_interface/ci_kheperaiv_wifi_actuator.h>
#include <argos3/plugins/robots/kheperaiv/control_interface/ci_kheperaiv_wifi_sensor.h>
#include <argos3/plugins/robots/kheperaiv/control_interface/ci_kheperaiv_ground_sensor.h>
#include <argos3/plugins/robots/kheperaiv/control_interface/ci_kheperaiv_proximity_sensor.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/datatypes/byte_array.h>

#include <mutex>
#include <atomic>
#include <unordered_map>

#include "algorithms/CollectivePerception.hpp"
#include "algorithms/SensorDegradationFilter.hpp"
#include "messages/RobotServerMessage.hpp"

using namespace argos;

class BayesCPFDiffusionController : public CCI_Controller
{
public:
    struct WheelTurningParams
    {
        /*
         * The turning mechanism.
         * The robot can be in three different turning states.
         */
        enum class TurningMechanism
        {
            NO_TURN = 0, // go straight
            SOFT_TURN,   // both wheels are turning forwards, but at different speeds
            HARD_TURN    // wheels are turning with opposite speeds
        };
        /*
         * Angular thresholds to change turning state.
         */
        CRadians HardTurnOnAngleThreshold;
        CRadians SoftTurnOnAngleThreshold;
        CRadians NoTurnAngleThreshold;
        /* Maximum wheel speed */
        Real MaxSpeed;
        TurningMechanism TurnMech;

        WheelTurningParams();
        void Init(TConfigurationNode &xml_node);
    };

    struct DiffusionParams
    {
        /*
         * Maximum tolerance for the proximity reading between
         * the robot and the closest obstacle.
         * The proximity reading is 0 when nothing is detected
         * and grows exponentially to 1 when the obstacle is
         * touching the robot.
         */
        Real Delta;
        /* Angle tolerance range to go straight. */
        CRange<CRadians> GoStraightAngleRange;
        /* Movement bounds in x-direction */
        CRange<Real> BoundsX;
        /* Movement bounds in y-direction */
        CRange<Real> BoundsY;

        /* Constructor */
        DiffusionParams();

        /* Parses the XML section for diffusion */
        void Init(TConfigurationNode &xml_node);
    };

    struct GroundSensorParams
    {
        bool IsSimulated = true;
        bool IsDynamic = false;
        UInt32 GroundMeasurementPeriodTicks = 0;
        Real LowestDegradedAccuracyLevel = 0.5;
        std::unordered_map<std::string, Real> ActualSensorAcc = {{"b", -1.0}, {"w", -1.0}};
        std::unordered_map<std::string, Real> InitialActualAcc = {{"b", -1.0}, {"w", -1.0}};
        std::unordered_map<std::string, Real> DegradationCoefficients = {{"drift", 0.0}, {"diffusion", -1.0}};
    };

    struct CommsParams
    {
        UInt32 CommsPeriodTicks = 0;
        Real SingleHopRadius = 0.0;
    };

    struct ARGoSServerParams
    {
        std::string Address;
        SInt32 Port = 0;
        size_t ServerToRobotMsgSize = 0; // messsage size coming from the server; MUST match the config file used for the loop functions
        size_t RobotToServerMsgSize = 0; // messsage size goint to the server; MUST match the config file used for the loop functions
    };

public:
    BayesCPFDiffusionController();

    virtual ~BayesCPFDiffusionController();

    virtual void Init(TConfigurationNode &xml_node);

    virtual void Reset();

    virtual void ControlStep();

    WheelTurningParams GetWheelTurningParams() const { return wheel_turning_params_; }

    DiffusionParams GetDiffusionParams() const { return diffusion_params_; }

    GroundSensorParams GetGroundSensorParams() const { return ground_sensor_params_; }

    CommsParams GetCommsParams() const { return comms_params_; }

    CollectivePerception::Params GetCollectivePerceptionParams() const { return *(collective_perception_algo_ptr_->GetParamsPtr()); }

    SensorDegradationFilter::Params GetSensorDegradationFilterParams() const { return *(sensor_degradation_filter_ptr_->GetParamsPtr()); }

    std::vector<Real> GetData() const;

    void UpdateAssumedSensorAcc(const std::unordered_map<std::string, Real> &updated_accuracies_map, const bool &initial = false)
    {
        sensor_degradation_filter_ptr_->GetParamsPtr()->AssumedSensorAcc = updated_accuracies_map;

        if (initial)
        {
            sensor_degradation_filter_ptr_->GetParamsPtr()->InitialAssumedAcc = updated_accuracies_map;
        }
    }

    inline void SetRNGSeed(const UInt32 &seed)
    {
        RNG_ptr_->SetSeed(seed);
        RNG_ptr_->Reset();
    }

    void ActivateDegradationFilter() { sensor_degradation_filter_ptr_->GetParamsPtr()->RunDegradationFilter = true; }

    void DeactivateDegradationFilter() { sensor_degradation_filter_ptr_->GetParamsPtr()->RunDegradationFilter = false; }

private:
    UInt32 ObserveTileColor();

    void ConnectToARGoSServer();

    void ListenToARGoSServer();

    void SendDataToARGoSServer();

    CVector2 ComputeDiffusionVector();

    void SetWheelSpeedsFromVector(const CVector2 &heading_vector);

    void EvolveSensorDegradation();

    void GetSelfPosition();

    std::vector<CollectivePerception::EstConfPair> GetNeighborMessages();

protected:
    /* Pointer to the positioning sensor */
    CCI_PositioningSensor *ci_positioning_sensor_ptr_;

    /* Pointer to the WiFi sensor */
    CCI_KheperaIVWiFiSensor *ci_wifi_sensor_ptr_;

    /* Pointer to the WiFi actuator */
    CCI_KheperaIVWiFiActuator *ci_wifi_actuator_ptr_;

    /* Pointer to the differential steering actuator */
    CCI_DifferentialSteeringActuator *ci_wheels_ptr_;

    /* Pointer to the ground sensor */
    CCI_KheperaIVGroundSensor *ci_ground_ptr_;

    /* Pointer to the proximity sensor */
    CCI_KheperaIVProximitySensor *ci_proximity_ptr_;

    /* Pointer to the random number generator */
    CRandom::CRNG *RNG_ptr_;

    /* Turning parameters */
    WheelTurningParams wheel_turning_params_;

    /* Diffusion parameters */
    DiffusionParams diffusion_params_;

    /* Ground sensor parameters */
    GroundSensorParams ground_sensor_params_;

    /* Communications parameters */
    CommsParams comms_params_;

    /* ARGoS server parameters */
    ARGoSServerParams argos_server_params_;

    /* Collective perception algorithm */
    std::shared_ptr<CollectivePerception> collective_perception_algo_ptr_;

    /* Sensor degradation filter */
    std::shared_ptr<SensorDegradationFilter> sensor_degradation_filter_ptr_;

    /* Messages received by the WiFi sensor */
    std::vector<CCI_KheperaIVWiFiSensor::SMessage> messages_vec_;

    /* Position of the robot based on the positioning sensor */
    CVector3 self_pose_;

    /* Socket for connecting to the ARGoS server */
    SInt32 socket_;

    /* Mutex to protect self_pose_ */
    std::mutex self_pose_mutex_;

    /* Flag to start operation */
    std::atomic<bool> start_flag_{false};

    /* Flag to initiate shutdown */
    std::atomic<bool> shutdown_flag_{false};

    Real assumed_degradation_drift_ = 0.0;

    Real prev_assumed_acc_ = 0.0;

    std::pair<Real, Real> averaged_deg_rates_and_fill_ratio_refs_ = {0.0, 0.0};

    std::string network_name_;

    size_t window_size_ = 0;

    std::deque<std::pair<Real, Real>> previous_degradation_rates_and_fill_ratio_references_;

    UInt64 tick_counter_ = 0;

    CRange<Real> standard_uniform_support_ = CRange<Real>(0.0, 1.0);

    /* Variables for sending robot data vector to the ARGoS server */
    CByteArray data_vec_byte_arr_;

    std::vector<Real> data_vec_;

    UInt8 *data_vec_send_buffer_;

    RobotServerMessage data_vec_msg_to_send_;

    ssize_t data_vec_remaining_size_;

    ssize_t data_vec_bytes_sent_;
};

#endif