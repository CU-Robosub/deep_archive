#ifndef _CONTROL_PLUGIN_HH_
#define _CONTROL_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"

namespace gazebo
{
    class ControlPlugin : public ModelPlugin
    {
        public: ControlPlugin() {}

        public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
          this->_model = _model;
          std::cerr << "\nThe sub plugin is attach to model[" <<
            _model->GetName() << "]\n";
            if (!ros::isInitialized())
            {
              int argc = 0;
              char **argv = NULL;
              ros::init(argc, argv, "gazebo_client",
                  ros::init_options::NoSigintHandler);
            }
            this->rosNode.reset(new ros::NodeHandle("gazebo_client"));
            ros::SubscribeOptions soY =
              ros::SubscribeOptions::create<std_msgs::Float32>(
                  "/" + _model->GetName() + "/vel_y",
                  1,
                  boost::bind(&ControlPlugin::OnRosMsgY, this, _1),
                  ros::VoidPtr(), &this->rosQueue);
            ros::SubscribeOptions soX =
            ros::SubscribeOptions::create<std_msgs::Float32>(
                "/" + _model->GetName() + "/vel_x",
                1,
                boost::bind(&ControlPlugin::OnRosMsgX, this, _1),
                ros::VoidPtr(), &this->rosQueue);
            ros::SubscribeOptions soZ =
            ros::SubscribeOptions::create<std_msgs::Float32>(
                "/" + _model->GetName() + "/vel_z",
                1,
                boost::bind(&ControlPlugin::OnRosMsgZ, this, _1),
                ros::VoidPtr(), &this->rosQueue);
            ros::SubscribeOptions soYr =
              ros::SubscribeOptions::create<std_msgs::Float32>(
                  "/" + _model->GetName() + "/rot_y",
                  1,
                  boost::bind(&ControlPlugin::OnRosMsgYr, this, _1),
                  ros::VoidPtr(), &this->rosQueue);
            ros::SubscribeOptions soXr =
            ros::SubscribeOptions::create<std_msgs::Float32>(
                "/" + _model->GetName() + "/rot_x",
                1,
                boost::bind(&ControlPlugin::OnRosMsgXr, this, _1),
                ros::VoidPtr(), &this->rosQueue);
            ros::SubscribeOptions soZr =
            ros::SubscribeOptions::create<std_msgs::Float32>(
                "/" + _model->GetName() + "/rot_z",
                1,
                boost::bind(&ControlPlugin::OnRosMsgZr, this, _1),
                ros::VoidPtr(), &this->rosQueue);
            ros::SubscribeOptions so =
            ros::SubscribeOptions::create<std_msgs::Float32MultiArray>(
                "/" + _model->GetName() + "/cmd",
                6,
                boost::bind(&ControlPlugin::OnRosMsg, this, _1),
                ros::VoidPtr(), &this->rosQueue);
            this->rosSubX = this->rosNode->subscribe(soX);
            this->rosSubY = this->rosNode->subscribe(soY);
            this->rosSubZ = this->rosNode->subscribe(soZ);
            this->rosSubXr = this->rosNode->subscribe(soXr);
            this->rosSubYr = this->rosNode->subscribe(soYr);
            this->rosSubZr = this->rosNode->subscribe(soZr);

            this->rosSub = this->rosNode->subscribe(so);

            this->rosQueueThread =
            std::thread(std::bind(&ControlPlugin::QueueThread, this));
        }
        //msg handlers for the sub
        public: void OnRosMsg(const std_msgs::Float32MultiArrayConstPtr &_msg)
        {
            this->xV = _msg->data[0];
            this->yV = _msg->data[1];
            this->zV = _msg->data[2];
            this->xR = _msg->data[3];
            this->yR = _msg->data[4];
            this->zR = _msg->data[5];
            this->SetVelocity();
            this->SetAngular();
        }
        public: void OnRosMsgX(const std_msgs::Float32ConstPtr &_msg)
        {
            this->xV = _msg->data;
            this->SetVelocity();
        }
        public: void OnRosMsgY(const std_msgs::Float32ConstPtr &_msg)
        {
            this->yV = _msg->data;
            this->SetVelocity();
        }
        public: void OnRosMsgZ(const std_msgs::Float32ConstPtr &_msg)
        {
            this->zV = _msg->data;
            this->SetVelocity();
        }
        public: void OnRosMsgXr(const std_msgs::Float32ConstPtr &_msg)
        {
            this->xR = _msg->data;
            this->SetAngular();
        }
        public: void OnRosMsgYr(const std_msgs::Float32ConstPtr &_msg)
        {
            this->yR = _msg->data;
            this->SetAngular();
        }
        public: void OnRosMsgZr(const std_msgs::Float32ConstPtr &_msg)
        {
            this->zR = _msg->data;
            this->SetAngular();
        }
        //velocity changing function
        public: void SetVelocity(){
            //this sets the velocity in the robot frame
            this->_model->SetLinearVel(this->_model->GetWorldPose().rot.RotateVector(math::Vector3(this->xV,this->yV,this->zV)));
        }
        public: void SetAngular(){
            //this sets the angle in the robot frame
            this->_model->SetAngularVel(this->_model->GetWorldPose().rot.RotateVector(math::Vector3(this->xR,this->yR,this->zR)));
        }

        //the model
        private:  physics::ModelPtr _model;
        private: void Update ( const common::UpdateInfo &_info){
            this->_model->SetLinearVel(this->_model->GetWorldPose().rot.RotateVector(math::Vector3(this->xV,this->yV,this->zV)));
            this->_model->SetAngularVel(this->_model->GetWorldPose().rot.RotateVector(math::Vector3(this->xR,this->yR,this->zR)));
        }

        private: void QueueThread()
        {
          static const double timeout = 0.01;
          while (this->rosNode->ok())
          {
            this->rosQueue.callAvailable(ros::WallDuration(timeout));
          }
        }
        /// \brief A node use for ROS transport
        private: std::unique_ptr<ros::NodeHandle> rosNode;

        //doubles to store velocities
        private: double xV;
        private: double yV;
        private: double zV;
        private: double xR;
        private: double yR;
        private: double zR;
        //Group of subscribers
        private: ros::Subscriber rosSubX;
        private: ros::Subscriber rosSubY;
        private: ros::Subscriber rosSubZ;
        private: ros::Subscriber rosSubXr;
        private: ros::Subscriber rosSubYr;
        private: ros::Subscriber rosSubZr;

        private: ros::Subscriber rosSub;
        private: ros::CallbackQueue rosQueue;

        /// \brief A thread the keeps running the rosQueue
        private: std::thread rosQueueThread;
    };

    // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
    GZ_REGISTER_MODEL_PLUGIN(ControlPlugin)
}
#endif
