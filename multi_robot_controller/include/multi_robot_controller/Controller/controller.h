#include<multi_robot_controller/Input/input_types.hpp>
#include<multi_robot_controller/Input/input_pose_odom.hpp>
#include<multi_robot_controller/Input/input_odom.hpp>
#include<multi_robot_controller/Input/input_pose_twist.hpp>
#include<multi_robot_controller/RigidMotion/constrained_rigid_motion_tf.h>
#include<multi_robot_controller/necessary_param_exeption.hpp>
#include<multi_robot_controller/input_alloc_exception.hpp>
#include<multi_robot_msgs/MetaData.h>
#include<std_srvs/Empty.h>
#include<tf/tf.h>

#define DEBUG_STATE(target_state)ROS_INFO_STREAM(target_state.pose.getOrigin().x()<<"\t"<<target_state.pose.getOrigin().y()<<"\t"<<target_state.pose.getOrigin().z()<<"\n"<<target_state.pose.getRotation().x()<<"\t"<<target_state.pose.getRotation().y()<<"\t"<<target_state.pose.getRotation().z()<<"\t"<<target_state.pose.getRotation().w())



class Controller
{
    public:
        Controller();
        
        Controller(ros::NodeHandle &nh);

        ~Controller(); 

        struct ControlVector
        {
            double v;                   ///<Linear velocity
            double omega;               ///<Angular velocity
            /**
             * @brief Construct a new Control Vector object with default parameters
             * 
             */
            ControlVector(){v=0.0;omega=0.0;}
            ControlVector(double v, double omega)
            {
                this->v=v;
                this->omega=omega;
            }
        };
        struct State
        {
            tf::Pose pose;
            tf::Vector3 lin_vel;
            tf::Vector3 ang_vel;
            State();
            State(tf::Pose pose,tf::Vector3 lin_vel,tf::Vector3 ang_vel);      
        };
    protected:
        virtual ControlVector calcControl(State current_state,State target_state)=0;
        virtual void publishMetaData();
        ros::Publisher meta_;
    
    private:
        bool enableCallback(std_srvs::EmptyRequest &req,std_srvs::EmptyRequest &res);
        bool disableCallback(std_srvs::EmptyRequest &req,std_srvs::EmptyRequest &res);
        std::unique_ptr<InputBase> allocInput(InputTypes target,ros::NodeHandle parameter); 

        bool enable_;
        ros::NodeHandle nh_;

        std::unique_ptr<InputBase> current_state_handler_;
        std::unique_ptr<InputBase> target_state_handler_;
        
        ros::Publisher pub_;
        ros::ServiceServer enable_srv_;
        ros::ServiceServer disable_srv_;
       

        State current_state_;
        State target_state_;

        ControlVector control_;

        ConstrainedRigidMotionTf rigid_motion_;
        ros::Timer control_scope_timer_;
        void controlScope(const ros::TimerEvent&);
        void publish();
   
       
    
};

