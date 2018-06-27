/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Hye-Jong KIM*/

#ifndef OMMANAGER_H_
#define OMMANAGER_H_

#include <unistd.h>
#include <WString.h>
#include <Eigen.h>       

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

typedef struct
{
  Eigen::Vector3f position;
  Eigen::Matrix3f orientation;
} Pose;

typedef struct
{
  float position;
  float velocity;
  float acceleration;
} State;






/////////////////////////////////////////Manipulator//////////////////////////////////////////

class Manipulator
{
  private:
  
    int8_t dof_;
    String name_;

  public:
    /////////////////func///////////////////
    Manipulator();
    ~Manipulator();
    void Init(int8_t dof, int8_t number_of_joint, int8_t number_of_link, int8_t number_of_tool);
    void Init(int8_t dof, int8_t number_of_joint, int8_t number_of_link);
    void SetDOF(int8_t dof);
    ////////////////////////////////////////
    class Joint
    {
      private:
        String name_;
        int8_t dxl_id_;
        int8_t number_;	

        float angle_;
        float velocity_;
        float acceleration_;

        Eigen::Vector3f position_;
        Eigen::Matrix3f orientation_;

      public:
        /////////////////func///////////////////
        Joint();
        ~Joint();
        void Init(String name, int8_t number, int8_t dxl_id);
        void SetAngle(float angle);
        void SetVelocity(float velocity);
        void SetAcceleration(float acceleration);
        float GetAngle();
        float GetVelocity();
        float GetAcceleration();
        void SetPosition(Eigen::Vector3f position);
        void SetOrientation(Eigen::Matrix3f orientation);
        Eigen::Vector3f GetPosition();
        Eigen::Matrix3f GetOrientation();
        Pose GetPose();
        ////////////////////////////////////////
    };

    class Link
    {
      private:
        String name_;
        float mass_;			
        float inertia_moment;

        Eigen::Vector3f center_position_;

      public:
        /////////////////func///////////////////
        Link();
        ~Link();
        void Init(String name, float mass, Eigen::Vector3f center_position, int8_t number_of_joint_in_link);
        float GetInertiaMoment();
        ////////////////////////////////////////

        class JointInLink
        {
          private:
            int8_t number_;
            
            Eigen::Vector3f relative_position_;
            Eigen::Matrix3f relative_orientation_;
          public:
            /////////////////func///////////////////
            JointInLink();
            ~JointInLink();

            void Init(int8_t joint_number, Eigen::Vector3f relative_position, Eigen::Vector3f axis);
            void Init(int8_t joint_number, Eigen::Vector3f relative_position, Eigen::Matrix3f relative_axis_matrix, Eigen::Vector3f axis);


            ////////////////////////////////////////
        };
    };
    class Tool
    {
      private:
        String tool_type_;
        int8_t number_;

        Eigen::Vector3f position_from_final_joint_;
        Eigen::Matrix3f orientation_from_final_joint_;

        Eigen::Vector3f position_;
        Eigen::Matrix3f orientation_;

      public:
        Tool();
        ~Tool();
        void Init(String tool_type, int8_t number, Eigen::Vector3f position_from_final_joint, Eigen::Matrix3f orientation_from_final_joint);
        void SetPosition(Eigen::Vector3f position);
        void SetOrientation(Eigen::Matrix3f orientation);
        Eigen::Vector3f GetPosition();
        Eigen::Matrix3f GetOrientation();
        Pose GetPose();

    };
};
#endif // OMMANAGER_H_

