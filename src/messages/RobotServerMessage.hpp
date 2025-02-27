#ifndef ROBOT_SERVER_MESSAGE_HPP
#define ROBOT_SERVER_MESSAGE_HPP

#include <cstring>

#include <argos3/core/utility/datatypes/byte_array.h>

using namespace argos;

struct RobotServerMessage
{
    UInt16 Size;
    CByteArray Payload;

    inline void PopulateMessage(const CByteArray &arr)
    {
        Payload = arr;

        Size = Payload.Size();
    }

    inline void Serialize(UInt8 *buffer) const
    {
        // Serialize the size of the message
        std::memcpy(buffer, &Size, sizeof(Size));

        // Serialize the payload
        std::memcpy(buffer + sizeof(Size), Payload.ToCArray(), Payload.Size());
    }

    inline RobotServerMessage Deserialize(const UInt8 *buffer)
    {
        RobotServerMessage msg;

        // Extract the size
        std::memcpy(&msg.Size, buffer, sizeof(Size));

        // Extract the payload
        msg.Payload = CByteArray(buffer + sizeof(Size), Size);

        return msg;
    }
};

#endif