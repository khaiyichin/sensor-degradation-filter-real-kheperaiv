#ifndef ROBOT_SERVER_MESSAGE_HPP
#define ROBOT_SERVER_MESSAGE_HPP

#include <cstring>

#include <argos3/core/utility/datatypes/byte_array.h>

using namespace argos;

/**
 * @brief Message struct for communication between the robot and the ARGoS server
 * This is used both by the controller and the loop functions in sensor-degradation-filter
 * https://github.com/khaiyichin/sensor-degradation-filter/blob/4289ea55fc6f90aa02018eea4325a8c009e8e07e/include/sensor_degradation_filter/loop_functions/RealKheperaIVExperimentLoopFunctions.hpp
 *
 */
class RobotServerMessage
{

public:
    inline void PopulateMessage(const CByteArray &arr)
    {
        Payload = arr;

        Size = Payload.Size();
        Size = Payload.Size(); // CByteArray size (i.e., how many bytes)
    }

    inline void Serialize(UInt8 *buffer) const
    {
        // Serialize the size of the message
        std::memcpy(buffer, &Size, sizeof(Size));

        // Serialize the payload
        std::memcpy(buffer + sizeof(Size), Payload.ToCArray(), Payload.Size());
    }

    inline void Deserialize(const UInt8 *buffer)
    {
        // Extract the size
        std::memcpy(&Size, buffer, sizeof(Size));

        // Extract the payload
        Payload = CByteArray(buffer + sizeof(Size), Size);
    }

    inline void CleanUp()
    {
        Size = 0;
        Payload.Clear();
    }

    inline CByteArray GetPayload() const { return Payload; }

    inline UInt16 GetSize() const { return Size; }

protected:
    UInt16 Size;

    CByteArray Payload;
};

#endif