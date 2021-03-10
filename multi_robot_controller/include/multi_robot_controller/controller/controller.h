#include<multi_robot_controller/input/input_types.hpp>
#include<multi_robot_controller/input/input_pose_odom.hpp>
#include<multi_robot_controller/input/input_odom.hpp>
#include<multi_robot_controller/input/input_pose_twist.hpp>
#include<multi_robot_controller/input/input_pose_twist_local.hpp>
#include<multi_robot_controller/input/input_pose_twist_stamped.hpp>
#include<multi_robot_controller/input/input_pose_twist_stamped_local.hpp>
#include<multi_robot_controller/rigid_motion/constrained_rigid_motion_tf.h>
#include<multi_robot_controller/necessary_param_exeption.hpp>
#include<multi_robot_controller/input_alloc_exception.hpp>
#include<multi_robot_msgs/MetaData.h>
#include<std_srvs/Empty.h>
#include<tf/tf.h>

#define DEBUG_STATE(target_state)ROS_INFO_STREAM(target_state.pose.getOrigin().x()<<"\t"<<target_state.pose.getOrigin().y()<<"\t"<<target_state.pose.getOrigin().z()<<"\n"<<target_state.pose.getRotation().x()<<"\t"<<target_state.pose.getRotation().y()<<"\t"<<target_state.pose.getRotation().z()<<"\t"<<target_state.pose.getRotation().w())



/**
 * @brief Class for controlling a formation of multiple mobile robots.
 * 
 *  Objects of this class contain the following ros parameter:
 * 
 * |Ros-Parameter   |Description        |
 * |----------------|-------------------|
 * |~target/input_type| Type of the target state topic.|
 * |~current/input_type| Type of the current state topic.|
 * |~rate            | Rate the controller is spinning with.|
 * |~topic_output     | Name of the output topic.|
 * 
 */
class Controller
{
    public:
        /**
         * @brief Construct a new Controller object
         * 
         * 
         */
        Controller();

        /**
         * @brief Construct a new Controller object with given Nodehandle
         * 
         * @param nh Nodehandle for resolving topics
         */
        
        Controller(ros::NodeHandle &nh);

        /**
         * @brief Destroy the Controller object
         * 
         */
        ~Controller(); 

        /**
         * @brief Defines the controllers output
         * 
         */
        struct ControlVector
        {
            double v;                   ///<Linear velocity of plattform
            double omega;               ///<Angular velocity of plattform
            /**
             * @brief Construct a new Control Vector object with default parameters
             * 
             */
            ControlVector(){v=0.0;omega=0.0;}
            /**
             * @brief Construct a new Control Vector object
             * 
             * @param v Linear velocity of plattform
             * @param omega Angular velocity of plattform
             */
            ControlVector(double v, double omega)
            {
                this->v=v;
                this->omega=omega;
            }
        };
        /**
         * @brief Defines the state representations within the Controller
         * 
         */
        struct State
        {
            tf::Pose pose;  //< Pose of the mobile robot in space
            tf::Vector3 lin_vel;    //<Linear velocity of the robot in space
            tf::Vector3 ang_vel;    //<Angular velocity of the robot in space
            
            /**
             * @brief Construct a new State object
             * 
             */
            State();
            /**
             * @brief Construct a new State object
             * 
             * @param pose Pose of the mobile robot in space
             * @param lin_vel Linear velocity of the robot in space
             * @param ang_vel Angular velocity of the robot in space
             */
            State(tf::Pose pose,tf::Vector3 lin_vel,tf::Vector3 ang_vel);      
        };
    protected:
        /**
         * @brief Control law fucntion. Determines the control vector by a given current and target state. Is implemented in derived Classes
         * 
         * @param current_state Current state of the controlled robot.
         * @param target_state State of the mobile robot to be reached.
         * @return ControlVector Commands to gain the target state from current state
         */
        virtual ControlVector calcControl(State current_state,State target_state)=0;
        /**
         * @brief Publishes the meta data of the controller. Thes are e.g. the current and target states and the control differences.
         * 
         */
        virtual void publishMetaData();


        ros::Publisher meta_;       //<Publisher for controllers meta data
        State control_diff_;        //<Difference between current and target state
    
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

