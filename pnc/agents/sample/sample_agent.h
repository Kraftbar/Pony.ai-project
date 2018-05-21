// Copyright @2018 Pony AI Inc. All rights reserved.

#pragma once

#include "pnc/simulation/vehicle_agent.h"

#include "common/proto/agent_status.pb.h"
#include "common/proto/geometry.pb.h"
#include "common/proto/vehicle_status.pb.h"
#include "common/utils/math/math_utils.h"
#include "pnc/simulation/vehicle_agent_factory.h"
#include "pnc/agents/sample/PID.h"


// PID PID PID
 PID pid=PID(.1,.01,0);

namespace sample {

// A sample vehicle agent for route 1
// This agent will always run in straight. It will accelerate to the speed slightly over 5m/s,
// and keeps running with the speed over 5m/s until reaches destination.

    class SampleVehicleAgent : public simulation::VehicleAgent {
    public:
        explicit SampleVehicleAgent(const std::string &name) : VehicleAgent(name) {}

        void Initialize(const interface::agent::AgentStatus & /* agent_status */) override {
            // Make sure there is dobules


            // PID PID PID
             pid.setOutputLimits(0,1);
            // pid.setOutputRampRate(4);

        }


        // doc:
        // styre bilen:
        //  command.
        //
        // om bilen
        //   agent_status.vehicle_status().
        //
        // Om verdenen:
        //  agent_status.simulation_status().
        //


        // ######################################################################
        // ######################################################################
        interface::control::ControlCommand RunOneIteration(
                const interface::agent::AgentStatus &agent_status) override
        {
            interface::control::ControlCommand command;
            // Vehicle's current position reaches the destination
            // if ( CalcDistanceDest(agent_status)< 3.0)
            if ( CalcDistanceDest(agent_status)< 7.0)
            {
               // position_reached_destination_ = true;
            }


            // IF I PRINT OUT THE VELOCITY GOES TO zero sometimes

            std::cout << CalcDistance(agent_status.vehicle_status().position(),
                                      agent_status.route_status().destination()) << std::endl;


            // when you drive in one 1m/s ~~ 3.3 m/s??
            // then one lane is ca. 44s
            // and one cross is 10s with angle 2.1
            double en_Lane =44;


            double sim_time= agent_status.simulation_status().simulation_time();
            if(sim_time>11+svinger*en_Lane && sim_time<21+svinger*en_Lane){
                command.set_steering_angle(2.1);
                harsvingt=1;
            }else{
                if(harsvingt==1){
                    svinger++;
                    harsvingt=0;
                }
                command.set_steering_angle(0);
            }

            // Vehicle's current velocity reaches 5 m/s
//            if (CalcVelocity(agent_status.vehicle_status().velocity()) > 5)
//            {
//                velocity_reached_threshold_ = true;
//            }

            if (position_reached_destination_)
            {
                command.set_brake_ratio(1.0);  // max is 1
            } else {
                if (!velocity_reached_threshold_)
                {
                    // PID PID PID
                    command.set_throttle_ratio(pid.getOutput(CalcVelocityCar(agent_status),1));
                    // command.set_throttle_ratio(0.3);
                }
            }





            PublishVariable("key1", "var1");
            PublishVariable("key2", "var2", utils::display::Color::Red());
            PublishVariable("key3", "var3", utils::display::Color::Red(), utils::display::Color::Green());

            return command;
        }
        // ######################################################################
        // ######################################################################




    private:
        double CalcDistance(const interface::geometry::Vector3d &position,
                            const interface::geometry::Point3D &destination)
        {
            double sqr_sum =
                    math::Sqr(position.x() - destination.x()) + math::Sqr(position.y() - destination.y());
            return std::sqrt(sqr_sum);
        }


        // distance to goal:
        // (x_pos-x_dest)^2+(x_pos-x_dest)^2
        double CalcDistanceDest(const interface::agent::AgentStatus &agent_status)
        {
            double sqr_sum =
                            math::Sqr(agent_status.vehicle_status().position().x()
                                      - agent_status.route_status().destination().x()) +
                            math::Sqr(agent_status.vehicle_status().position().y()
                                      - agent_status.route_status().destination().y());
            return std::sqrt(sqr_sum);
        }




        double CalcVelocity(const interface::geometry::Vector3d &velocity)
        {
            double sqr_sum = math::Sqr(velocity.x()) + math::Sqr(velocity.y());
            return std::sqrt(sqr_sum);
        }
        // Vel of car
        // x^2 + y^2
        double CalcVelocityCar(const interface::agent::AgentStatus &agent_status)
        {
            double sqr_sum = math::Sqr(agent_status.vehicle_status().velocity().x()) + math::Sqr(agent_status.vehicle_status().velocity().y());
            return std::sqrt(sqr_sum);
        }







        // Whether vehicle's current position reaches the destination
        bool position_reached_destination_ = false;
        // Whether vehicle's current velocity reaches 5 m/s
        bool velocity_reached_threshold_ = false;

        bool harsvingt=false;
        double svinger=1;
    };

}  // namespace sample
