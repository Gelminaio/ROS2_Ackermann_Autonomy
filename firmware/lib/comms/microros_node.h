#pragma once

#include <Arduino.h>

namespace comms
{
    // runs entirely inside comms_task on core 0, isolated from the control loop.
    // ff the agent disconnects, the node attempts to reconnect without
    // blocking motor control and safety tasks on core 1.
    // ============================================================================
    class MicroRosNode
    {
    public:
        // initialize transport, node, pubs/subs
        bool begin();

        // called periodically from comms_task. Handles executor spin and
        // publishing. Non-blocking with short timeouts.
        void spinOnce();

    private:
        bool createEntities();
        void destroyEntities();
        void publishJointStates();
        void publishImu();
        void publishSteering();

        bool agent_connected_ = false;
        uint32_t last_publish_ms_ = 0;
    };
}