#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/sensors.hh>
#include <stdio.h>

namespace gazebo
{
    class MoveForeward : public ModelPlugin
    {
        public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
                {
                    // Store the pointer to the model
                    this->model = _parent;

                    // Listen to the update event. This event is broadcast every
                    // simulation iteration.
                    if (this->LoadParams(this->model->GetSDF()))
                    {
                        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                                boost::bind(&MoveForeward::OnUpdate, this, _1));
                    }

                }


        public: bool LoadParams(sdf::ElementPtr _sdf)
                {
                    sensors::SensorPtr sensor =
                        sensors::SensorManager::Instance()->GetSensor("laser");

                    this->laser = boost::dynamic_pointer_cast<sensors::RaySensor>(sensor);
                    if (!this->laser)
                    {
                        gzerr << "Could not load laser\n";
                        return false;
                    }

                    this->leftWheel = this->model->GetJoint("left_wheel_hinge");
                    this->rightWheel = this->model->GetJoint("right_wheel_hinge");

                    if (!this->leftWheel || !this->rightWheel)
                    {
                        gzerr << "Could not load wheel joint elements\n";
                        return false;
                    }

                    return true;
                }
                // Called by the world update start event
        public: void OnUpdate(const common::UpdateInfo & /*_info*/)
                {
                    // apply force to the wheels
                    //
                        this->leftWheel->SetForce(0, 0.1);
                        this->rightWheel->SetForce(0, 0.1);
                }

                // Pointer to the model
        private: physics::ModelPtr model;

                 // Pointer to the update event connection
        private: event::ConnectionPtr updateConnection;

        private: sensors::RaySensorPtr laser;
        private: physics::JointPtr leftWheel;
        private: physics::JointPtr rightWheel;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(MoveForeward)
}
