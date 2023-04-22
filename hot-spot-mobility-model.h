#ifndef HOT_SPOT_MOBILITY_MODEL_H
#define HOT_SPOT_MOBILITY_MODEL_H

#include "constant-velocity-helper.h"
#include "mobility-model.h"
#include "position-allocator.h"
#include "ns3/ptr.h"
#include "ns3/random-variable-stream.h"


namespace ns3 {

    class HotSpotMobilityModel : public MobilityModel {

        public:

            static TypeId GetTypeId(void);
            
        
        protected:

            virtual void DoInitialize(void);

        private:

            void BeginWalk(void);
            void DoInitializePrivate(void);
            virtual Vector DoGetPosition(void) const;
            virtual void DoSetPosition(const Vector &position);
            virtual Vector DoGetVelocity(void) const;
            virtual int64_t DoAssignStreams(int64_t);
            void createHotspots(std::string);

            ConstantVelocityHelper m_helper;
            //Ptr<PositionAllocator> m_position;
            Ptr<RandomVariableStream> m_speed;
            Ptr<RandomVariableStream> m_pause;
            EventId m_event;

            //int num_hotspots;
            int num_waypoints;
            //Ptr<std::vector<Vector>> hotspots;
            std::vector<ns3::Vector> hotspots;
            Ptr<UniformRandomVariable> x = CreateObject<UniformRandomVariable>();
            Vector nextLocation;
            Vector nextDestination;
            int currJump = 0;
            int curr_index;
            int next_index;
            
    };

}

#endif