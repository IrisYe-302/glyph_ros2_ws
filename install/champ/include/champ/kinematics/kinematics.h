/*
Copyright (c) 2019-2020, Juan Miguel Jimeno
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef KINEMATICS_H
#define KINEMATICS_H

#ifdef __unix__
    #include <cmath>
    using namespace std;
#endif

#include <macros/macros.h>
#include <geometry/geometry.h>
#include <quadruped_base/quadruped_base.h>

namespace champ
{
    namespace
    {
        constexpr float kGo2ThighMin = -1.5708f;
        constexpr float kGo2ThighMax = 3.4907f;
        constexpr float kGo2CalfMin = -2.7227f;
        constexpr float kGo2CalfMax = -0.83776f;
    }

    class Kinematics
    {
        champ::QuadrupedBase *base_;
        
        public:
            Kinematics(champ::QuadrupedBase &quadruped_base):
                base_(&quadruped_base)
            {
            }

            void inverse(float (&joint_positions)[12], geometry::Transformation (&foot_positions)[4])
            {
                float calculated_joints[12];

                for(unsigned int i = 0; i < 4; i++)
                {
                    inverse(calculated_joints[(i*3)], calculated_joints[(i*3) + 1], calculated_joints[(i*3) + 2], *base_->legs[i], foot_positions[i]);
                    
                    //check if any leg has invalid calculation, if so disregard the whole plan
                    if(isnan(calculated_joints[(i*3)]) || isnan(calculated_joints[(i*3) + 1]) || isnan(calculated_joints[(i*3) + 2]))
                    {
                        return;
                    }
                }
                
                for(unsigned int i = 0; i < 12; i++)
                {
                    joint_positions[i] = calculated_joints[i];
                }
            }

            static void inverse(float &hip_joint, float &upper_leg_joint, float &lower_leg_joint, 
                                champ::QuadrupedLeg &leg, geometry::Transformation &foot_position)
            {
                geometry::Transformation temp_foot_pos = foot_position;

                float l0 = 0.0f;

                for(unsigned int i = 1; i < 4; i++)
                {
                    l0 += leg.joint_chain[i]->y();
                }

                float l1 = -sqrtf(pow(leg.lower_leg.x(), 2) + pow(leg.lower_leg.z(), 2));
                float ik_alpha = acosf(leg.lower_leg.x() / l1) - (M_PI / 2); 

                float l2 = -sqrtf(pow(leg.foot.x(), 2) + pow(leg.foot.z(), 2));
                float ik_beta = acosf(leg.foot.x() / l2) - (M_PI / 2); 

                float x = temp_foot_pos.X();
                float y = temp_foot_pos.Y();
                float z = temp_foot_pos.Z();
            
                hip_joint = -(atanf(y / z) - ((M_PI/2) - acosf(-l0 / sqrtf(pow(y, 2) + pow(z, 2)))));

                temp_foot_pos.RotateX(-hip_joint);
                temp_foot_pos.Translate(-leg.upper_leg.x(), 0.0f, -leg.upper_leg.z());

                x = temp_foot_pos.X();
                y = temp_foot_pos.Y();
                z = temp_foot_pos.Z();

                //reachability check
                float target_to_foot = sqrtf(pow(x, 2) + pow(z,2));
                if(target_to_foot >= (abs(l1) + abs(l2)))
                    return;

                //source: https://robotacademy.net.au/lesson/inverse-kinematics-for-a-2-joint-robot-arm-using-geometry/
                const float knee_term = acosf((pow(z, 2) + pow(x, 2) - pow(l1 ,2) - pow(l2 ,2)) / (2 * l1 * l2));

                auto solve_branch = [&](int branch_direction, float &upper_out, float &lower_out)
                {
                    lower_out = branch_direction * knee_term;
                    upper_out = (atanf(x / z) - atanf((l2 * sinf(lower_out)) / (l1 + (l2 * cosf(lower_out)))));
                    lower_out += ik_beta - ik_alpha;
                    upper_out += ik_alpha;

                    if(branch_direction < 0)
                    {
                        if(upper_out < 0)
                        {
                            upper_out = upper_out + M_PI;
                        }
                    }
                    else
                    {
                        if(upper_out > 0)
                        {
                            upper_out = upper_out + M_PI;
                        }
                    }
                };

                float preferred_upper = 0.0f;
                float preferred_lower = 0.0f;
                float alternate_upper = 0.0f;
                float alternate_lower = 0.0f;

                solve_branch(leg.knee_direction(), preferred_upper, preferred_lower);
                solve_branch(-leg.knee_direction(), alternate_upper, alternate_lower);

                const bool preferred_valid =
                    preferred_upper >= kGo2ThighMin && preferred_upper <= kGo2ThighMax &&
                    preferred_lower >= kGo2CalfMin && preferred_lower <= kGo2CalfMax;
                const bool alternate_valid =
                    alternate_upper >= kGo2ThighMin && alternate_upper <= kGo2ThighMax &&
                    alternate_lower >= kGo2CalfMin && alternate_lower <= kGo2CalfMax;

                if(preferred_valid || !alternate_valid)
                {
                    upper_leg_joint = preferred_upper;
                    lower_leg_joint = preferred_lower;
                }
                else
                {
                    upper_leg_joint = alternate_upper;
                    lower_leg_joint = alternate_lower;
                }
            }

            static void forward(geometry::Transformation foot_position, const champ::QuadrupedLeg &leg, 
                                const float upper_leg_theta, 
                                const float lower_leg_theta )
            {
                foot_position.Translate(leg.foot.x(), leg.foot.y(), leg.foot.z());
                foot_position.RotateY(lower_leg_theta);          

                foot_position.Translate(leg.lower_leg.x(), leg.lower_leg.y(), leg.lower_leg.z());
                foot_position.RotateY(upper_leg_theta);   

                foot_position.Translate(leg.upper_leg.x(), leg.upper_leg.y(), leg.upper_leg.z());
            }

            static void forward(geometry::Transformation foot_position, const champ::QuadrupedLeg &leg, 
                                const float hip_theta, 
                                const float upper_leg_theta, 
                                const float lower_leg_theta)
            {
                foot_position = Identity<4,4>();

                foot_position.Translate(leg.foot.x(), leg.foot.y(), leg.foot.z());
                foot_position.RotateY(lower_leg_theta);          

                foot_position.Translate(leg.lower_leg.x(), leg.lower_leg.y(), leg.lower_leg.z());
                foot_position.RotateY(upper_leg_theta);   

                foot_position.Translate(leg.upper_leg.x(), leg.upper_leg.y(), leg.upper_leg.z());
                foot_position.RotateY(hip_theta);   

                foot_position.Translate(leg.hip.x(), leg.hip.y(), leg.hip.z());
            }

            static void transformToHip(geometry::Transformation &foot_position, const champ::QuadrupedLeg &leg)
            {
                foot_position.Translate(-leg.hip.x(), -leg.hip.y(), -leg.hip.z());
            }

            static void transformToBase(geometry::Transformation &foot_position, const champ::QuadrupedLeg &leg)
            {
                foot_position.Translate(leg.hip.x(), leg.hip.y(), leg.hip.z());
            }
    };
}

#endif
