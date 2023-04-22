#include <cmath>
#include "ns3/simulator.h"
#include "ns3/random-variable-stream.h"
#include "ns3/pointer.h"
#include "ns3/string.h"
#include "hot-spot-mobility-model.h"
#include <sstream>
#include <stdio.h>
#include <string.h>
#include "ns3/uinteger.h"
#include "ns3/double.h"


//extern std::vector<Vector> ap_loc;

namespace ns3 {

    NS_OBJECT_ENSURE_REGISTERED(HotSpotMobilityModel);

    TypeId HotSpotMobilityModel::GetTypeId(void) {
        static TypeId tid = TypeId
        ("ns3::HotSpotMobilityModel")
            .SetParent<MobilityModel>()
            .SetGroupName("Mobility")
            .AddConstructor<HotSpotMobilityModel>()
            .AddAttribute("Speed", "A random variable used to pick the speed of a random waypoint model", StringValue("ns3::UniformRandomVariable[Min=4.0|Max=6.0]"),
                MakePointerAccessor(&HotSpotMobilityModel::m_speed),
                MakePointerChecker<RandomVariableStream>())
            .AddAttribute("Pause", "A random variable used to pick the pause of a random waypoint model", StringValue("ns3::ConstantRandomVariable[Constant=2.0]"),
                MakePointerAccessor(&HotSpotMobilityModel::m_pause),
                MakePointerChecker<RandomVariableStream>())
            // .AddAttribute("NumHotspots", "Number of Hotspots", IntegerValue(5),
            //     MakeIntegerAccessor(&HotSpotMobilityModel::num_hotspots),
            //     MakeIntegerChecker<int>())
            .AddAttribute("Hotspots", "Vector of Hotspots", StringValue("0,0,0 80,80,0 0,80,0 80,0,0"),
                MakeStringAccessor(&HotSpotMobilityModel::createHotspots),
                MakeStringChecker())
            .AddAttribute("Waypoints", "Number of Waypoints", UintegerValue(2),
                MakeUintegerAccessor(&HotSpotMobilityModel::num_waypoints),
                MakeUintegerChecker<uint32_t>());
            
            // .AddAttribute("Hotspots", "Hotspot Locations", PointerValue(),
            //     MakePointerAccessor(&HotSpotMobilityModel::hotspots),
            //     MakePointerChecker<>)

        return tid;
    }

    // String should look like 0,0,0 1,1,1 2,2,2  ...
    void HotSpotMobilityModel::createHotspots(std::string locations) {

        std::string loc_string = locations;
        
        size_t pos = 0;

        std::string delim1 = " ";
        std::string val;


        while ((pos = loc_string.find(delim1)) != std::string::npos) {
            //NS_LOG_UNCOND(pos);

            val = loc_string.substr(0, pos);

            std::string loc;
            size_t pos2 = 0;

            std::string locs[3];
            int counter = 0;
            while ((pos2 = val.find(',')) != std::string::npos) {
                loc = val.substr(0, pos2);

                //NS_LOG_UNCOND(loc);

                locs[counter] = loc;
                counter ++;
                val.erase(0, pos2 + 1);
            } 

            //NS_LOG_UNCOND(locs[1]);
            Vector location = Vector(std::stod(locs[0]), std::stod(locs[1]), 0);
            
            
            hotspots.push_back(location);
            
            
            //NS_LOG_UNCOND(location);
            loc_string.erase(0, pos + delim1.length());

            //NS_LOG_UNCOND(loc_string);

        }
        
        
        // char * loc_string = new char[locations.length() + 1];
        // strcpy(loc_string, locations.c_str());
        // char * p = std::strtok(loc_string, " ");

        // while(p != NULL) {
            
        //     char * c = std::strtok(p, ",");

        //     char * vals = new char[3];
        //     int counter = 0;
        //     while (c != NULL || counter < 3) {
        //         vals[counter] = *c;
        //         counter++;
        //     }
        //     //Vector location = Vector(double(p[1]) - '0', double(p[3]) - '0', double(p[5]) - '0');
        //     Vector location = Vector(double(vals[0]) - '0', double(vals[1]) - '0', double(vals[2]) - '0');
        //     NS_LOG_UNCOND(location);
        //     hotspots.push_back(location);
        //     p = std::strtok(NULL, " ");
        // }

        

    }

    void HotSpotMobilityModel::BeginWalk(void) {
        m_helper.Update();
        Vector m_current = m_helper.GetCurrentPosition();

        
        if (currJump < num_waypoints) {
            
            //NS_LOG_UNCOND(m_current);
            
            if (m_current.x < nextDestination.x) {
                x->SetAttribute("Min", DoubleValue(m_current.x + .1));
                x->SetAttribute("Max", DoubleValue((m_current.x + nextDestination.x) / 2));
            }
            else {
                x->SetAttribute("Min", DoubleValue((m_current.x + nextDestination.x) / 2));
                x->SetAttribute("Max", DoubleValue(m_current.x));
            }


            if (nextDestination.x - m_current.x == 0) {

                // Change for diagonal so if top right corner so m_current.y is either min or max, not only min
                if (m_current.y < nextDestination.y) {
                    x->SetAttribute("Min", DoubleValue(m_current.y + .1));
                    x->SetAttribute("Max", DoubleValue((m_current.y + nextDestination.y) / 2));
                }
                else {
                    x->SetAttribute("Min", DoubleValue((m_current.y + nextDestination.y) / 2));
                    x->SetAttribute("Max", DoubleValue(m_current.y));
                }
                
                nextLocation = Vector(m_current.x, x->GetValue(), 0);

                //NS_LOG_UNCOND("Going vertical");
                //NS_LOG_UNCOND(nextLocation);
            }
            else if (nextDestination.y - m_current.y == 0) {
                
                nextLocation = Vector(x->GetValue(), m_current.y, 0);

                //NS_LOG_UNCOND("Going horizontal");
                //NS_LOG_UNCOND(nextLocation);
            }
            else {
                
                double x_val = x->GetValue();
                double y_val;
                
                double slope = (nextDestination.y - m_current.y)/(nextDestination.x - m_current.x);
                double intercept = m_current.y - (slope * m_current.x);

                y_val = slope * x_val + intercept;

                nextLocation = Vector(x_val, y_val, 0);

                //NS_LOG_UNCOND("Diag");
                //NS_LOG_UNCOND(nextLocation);

            }
            
            

            //NS_LOG_UNCOND(nextLocation);

            currJump++;
            
        }
        else {
            currJump = 0;
            nextLocation = nextDestination;
            curr_index = next_index;

            x->SetAttribute("Min", DoubleValue(0.0));
            x->SetAttribute("Max", DoubleValue(hotspots.size() - 1));//(num_hotspots + 1));

            next_index = x->GetInteger();

            while(next_index == curr_index) {
                next_index = x->GetInteger();
            }

            nextDestination = hotspots[next_index];
        }

        double speed = m_speed->GetValue();
        double dx = (nextLocation.x - m_current.x);
        double dy = (nextLocation.y - m_current.y); 

        double k = speed / std::sqrt(dx * dx + dy * dy);

        m_helper.SetVelocity(Vector(k * dx, k * dy, 0));
        m_helper.Unpause();

        //NS_LOG_UNCOND(CalculateDistance(nextLocation, m_current));
        Time travelDelay = Seconds(CalculateDistance(nextLocation, m_current) / speed);

        
        m_event.Cancel();
        m_event = Simulator::Schedule(travelDelay, &HotSpotMobilityModel::DoInitializePrivate, this);
        NotifyCourseChange();
        
    }

    void HotSpotMobilityModel::DoInitialize(void) {
        
        // if (hotspots.size() < num_hotspots) {
        //     // x->SetAttribute("Min", (DoubleValue)(0.0));
        //     // x->SetAttribute("Max", (DoubleValue)(80.0));

        //     // // Initialize hot spots
        //     // for (int i = 0; i < num_hotspots; ++i) {
        //     //     Vector position = Vector(x->GetValue(), x->GetValue(), 0);
        //     //     hotspots.add(position);
        //     // }

        // }

        x->SetAttribute("Min", (DoubleValue)(0.0));
        x->SetAttribute("Max", (DoubleValue)(hotspots.size() - 1));//num_hotspots));
        
        // for (int i = 0; i < hotspots.size(); ++i) {
        //     NS_LOG_UNCOND(hotspots[i]);
        // }

        curr_index = x->GetInteger();

        //NS_LOG_UNCOND(curr_index);
        //NS_LOG_UNCOND(hotspots.size());
        m_helper.SetPosition(hotspots[curr_index]);

        next_index = x->GetInteger();
        //NS_LOG_UNCOND(next_index);

        while(next_index == curr_index) {
            next_index = x->GetInteger();
        }

        nextDestination = hotspots[next_index];

        //NS_LOG_UNCOND(nextDestination);

        DoInitializePrivate();

        MobilityModel::DoInitialize();
    }

    void HotSpotMobilityModel::DoInitializePrivate(void) {

        //m_helper.Update();
        m_helper.Pause();
        Time pause = Seconds(m_pause->GetValue());
        m_event = Simulator::Schedule(pause, 
            &HotSpotMobilityModel::BeginWalk, this);
        NotifyCourseChange();
        m_helper.Unpause();

    }


    Vector HotSpotMobilityModel::DoGetPosition(void) const {

        m_helper.Update();
        return m_helper.GetCurrentPosition();
    }

    void HotSpotMobilityModel::DoSetPosition(const Vector &position) {
        
        m_helper.SetPosition(position);
        Simulator::Remove(m_event);
        m_event = Simulator::ScheduleNow(&HotSpotMobilityModel::DoInitializePrivate, this);
    }
    
    Vector HotSpotMobilityModel::DoGetVelocity(void) const {
        return m_helper.GetVelocity();
    }

    int64_t HotSpotMobilityModel::DoAssignStreams(int64_t stream) {

        return 0;

    }

    


    // Pass in vector of hotspot locations
    // Choose m points 1-5 - Default is 2
    
    // Choose randomly from vector of hot spot location and choose next location for that point
    // Generate 2 random locations on the line to destination
    
    // Choose closest waypoint and go at constant velocity to it
    // Pause when you get to waypoint

    // Go to next waypoint, pause

    // Go to final destination

    // Choose new destination

    // Set velocity to something, default is 3

    // S ------x----------x------D
    // --------P----------P------P

    // Run for like 100s

}





// Choose your AP Node Locations
// Save locations in a vector - {(1,2,3), (6,7,8), ...} - > Outside of mobility model - Decided randomly or selected by us


// PASS INTO MM
// Using those locations, set locations as hot spot for mobility model - Hotspot is the strongest place

// Start each node at random hot spot location and have them move


// Outside of model
// Get the node location at specific time and analyze data rate from closest hot spot location
// Emulates locations of no wifi signal