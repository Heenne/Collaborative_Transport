#include<multi_robot_controller/input_types.hpp>
#include<multi_robot_controller/input_pose_odom.hpp>
#include<multi_robot_controller/input_odom.hpp>
#include<multi_robot_controller/input_pose_twist.hpp>
#include<multi_robot_controller/constrained_rigid_motion_tf.h>
#include<multi_robot_controller/necessary_param_exeption.hpp>
#include<tf/tf.h>
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
            State operator=(State s);   
        };
    protected:
        virtual ControlVector calcControl(State target_state,State current_state)=0;
    private:
        ros::NodeHandle nh_;

        InputBase* current_state_handler_;
        InputBase* target_state_handler_;
        
        ros::Publisher pub_;

        State current_state_;
        State target_state_;

        ControlVector control_;

        ConstrainedRigidMotionTf rigid_motion_;
        ros::Timer control_scope_timer_;
        void controlScope(const ros::TimerEvent&);
        void publish();
   
       
    
};

