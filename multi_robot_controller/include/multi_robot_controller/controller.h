#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <std_srvs/Empty.h>
#include <multi_robot_msgs/ControlData.h>
#include <multi_robot_msgs/ControlState.h>
#include <multi_robot_msgs/ControlVector.h>
#include <multi_robot_msgs/ControlDifference.h>
#include <multi_robot_msgs/SetInitialPose.h>
#include <multi_robot_msgs/SetParameter.h>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/mean.hpp>


#include <math.h>
#include <stdio.h>


#define PARAM_PUBISH_TF "publish_tf"

#define PARAM_REFERENCE_POSE "reference"
#define PARAM_TARGET_VEL "topic_target_velocity"
#define PARAM_TARGET_STATE "topic_target_state"
#define PARAM_TARGET_ODOM "topic_target_odometry"
#define PARAM_CURRENT_ODOM "topic_current_odometry"
#define PARAM_CURRENT_STATE "topic_current_state"


#define PARAM_COORD "coord"
#define PARAM_WORLD_FRAME "world_frame"
#define PARAM_TYPE "controller_type"
#define PARAM_THRESHOLD "thresh"

#define PARAM_LYAPUNOV "lyapunov"
#define PARAM_ANG_DIST "angle_distance"

using namespace boost::accumulators;

/**
 * @brief Provides a easy to use implementation of a generic mobile robot controller.
 */
class Controller{
    public:
        typedef tf::Vector3 VelocityCartesian;                      ///<Defines VelocityCartesian wich is used to stores cartesian defined velocities
        typedef tf::Vector3 PositionCartesian;                      ///<Defines PositionCartesian wich is used to store a cartesian defined position
        typedef tf::Transform ControlDifference;                    ///<Defines ControlDifference wich is a transformation from the current state in the target state
        

        /**
         * @brief Specifies the different implemented control laws
         * 
         */
        enum ControllerType
        {
            disable=0,
            pseudo_inverse=1,   /**< Control law is based on pseudo inveres the input for generate a output (least squares optimisation) */
            lypanov=2,          /**< Control law is based on the lyapunov approache. Output is determined from a lyapunov stable function */
            angle_distance=3,    /**< Control law based on linear approach in angle and distance respectively */
        };
        
        /**
         * @brief Specifies the parameters needed for the lyapunov based control law
         * 
         */
        struct LyapunovParameter
        {
            float kx;           ///< Control gain in x-direction 
            float ky;           ///< Control gain in y-direction  
            float kphi;         ///< Control gain in theta-direction

            /**
             * @brief Construct a new Lyapunov Parameter object with default Parameter
             * 
             */
            LyapunovParameter(){kx=0.0;ky=0.0;kphi=0.0;};
            /**
             * @brief Construct a new Lyapunov Parameter object with given parameters
             * 
             * @param kx    ///< Gain in kartesian x direction
             * @param ky    ///<Gain in kartesian y direction
             * @param kphi  ///<Gain in phi direction (around the z axis)
             */
            LyapunovParameter(float kx,float ky,float kphi){this->kx=kx;this->ky=ky;this->kphi=kphi;}
            /**
             * @brief Construct a new Lyapunov Parameter object with given parameters by vector
             * 
             * @param param vector that must contain the three object parameters
             */
            LyapunovParameter(std::vector<float> param)
            {
                if(param.size()!=3)
                {
                    throw std::invalid_argument("Wrong number of LyapunovParameters");
                }
                else
                {
                    this->kx=param[0];
                    this->ky=param[1];
                    this->kphi=param[2];
                }
            }
                
        };

        /**
         * @brief Specifies the Parameters of the angle distance based control law
         * 
         */
        struct AngleDistanceParameter
        {
            float angular_gain; ///< Control gain in phi direction  
            float linear_gain;  ///< Control gain in l direction   
            float d;    ///< parameter for collision avoidance
            /**
             * @brief Construct a new Angle Distance Parameter object, with default parameters
             * 
             */
            AngleDistanceParameter(){angular_gain=0.0;linear_gain=0.0;d=0.0;};
            /**
             * @brief Construct a new Angle Distance Parameter with given parameters
             * 
             * @param angular_gain  ///<Gain in Angular direction (phi/psi direction)
             * @param linear_gain   ///<Gain in linear direction (distance direction)
             * @param d     ///<Collision avoidance paramtere
             */
            AngleDistanceParameter(float angular_gain,float linear_gain,float d){this->angular_gain=angular_gain;this->linear_gain=linear_gain;this->d=d;}
            /**
             * @brief Construct a new Angle Distance Parameter from a given vector of parameters
             * 
             * @param param vector that must contain the three parameters of the object
             */
            AngleDistanceParameter(std::vector<float> param)
            {
                if(param.size()!=3)
                {
                    throw std::invalid_argument("Wrong number of AngleDistanceParameters");
                }
                else
                {
                    this->angular_gain=param[0];
                    this->linear_gain=param[1];
                    this->d=param[2];
                }
            }
        };

        /**
         * @brief A struct that holds linear and angular velocity of the Robot since this are the control variable for kinematic tracking control
         * 
         */
        struct ControlVector
        {
            double v;                   ///<Linear velocity
            double omega;               ///<Angular velocity
            /**
             * @brief Construct a new Control Vector object with default parameters
             * 
             */
            ControlVector(){v=0.0;omega=0.0;}
        };
        typedef ControlVector VelocityEulerian;                     ///<Defines VelocityEulerian wich is used to store Velocities wich are defined locally in the moved base system

        /**
         * @brief Holds the System state combined by the pose of the system, the cartesian velocity and the angular velocity
         * 
         */
        struct ControlState
        {
            tf::Pose pose;                  ///<Complete pose of the robot in 3 dimensional space
            VelocityCartesian velocity;     ///<Velocity in cartesian space
            double angular_velocity;        ///<Angular velocity around z-axis
            /**
             * @brief Construct a new Control State object with default memebers
             * 
             */
            ControlState()
            {
                pose=tf::Pose(tf::createIdentityQuaternion(),tf::Vector3(0.0,0.0,0.0));
                velocity=tf::Vector3(0.0,0.0,0.0);
                angular_velocity=0.0;
            }
        };
        /**
         * @brief Template for a buffer of a given size and a given type
         * 
         * @tparam T type if the values to be buffered
         */
        template <typename  T>
        struct Buffer
        {   
            /**
             * @brief Construct a new Buffer object with default size
             * 
             */
            Buffer():size_(1),
                    counter_(0)
            {}
            /**
             * @brief Construct a new Buffer object of given size
             * 
             * @param size size of the buffer (number of values to be buffered)
             */
            Buffer(int size):size_(size),
                             counter_(0)
            {}; 
            /**
             * @brief Insert an element into the buffer
             * 
             * @param element Element of type T to be buffered
             */
            void insert(T element)
            {
            
                if(counter_>=buffer_.size())
                {
                    buffer_.push_back(element);
                }
                else
                {
                    buffer_[counter_]=element;
                }
                if(counter_+1==size_)
                {
                    counter_=0;
                }
                else
                {
                    counter_++;
                }               
            }
            /**
             * @brief Get all currently buffered values
             * 
             * @return std::vector<T> vector of values within the buffer
             */
            std::vector<T> get()
            {
                return this->buffer_;
            }
            int size_;      ///<Size of the buffer
            int counter_;   ///<Current insert index
            std::vector<T> buffer_; ///<Buffered values

        };


         /*Konstruktor and Destruktor###################################################################################################################
        ##################################################################################################################################################*/
       

        /**
         * @brief Create the Controller object with given parameters
         * 
         * @param name Name of the controller
         * @param nh Global nodehandle
         * @param nh_topics Nodehandle to handle topic namepsaces
         * @param nh_parameters Nodehandle to handle parameter namepaces
         */
        Controller( std::string name,
                    ros::NodeHandle nh=ros::NodeHandle("~"),
                    ros::NodeHandle nh_topics=ros::NodeHandle("~"),
                    ros::NodeHandle nh_parameters=ros::NodeHandle("~"));
        /**
         * @brief Destroy the Controller object
         * 
         */
        ~Controller();


       
        
        /*Setter and parameter methods ###################################################################################################################
        ##################################################################################################################################################*/
        
        /**
         * @brief Set the name of object
         * 
         * @param name Name to be set
         */
        void setName(std::string name);       


        /**
         * @brief Set the world frame name all references are given in
         * 
         * @param frame Name of the world frame
         */
        void setWorldFrame(std::string frame);

        /**
         * @brief Set the Reference pose
         * 
         * @param pose pose to be set
         */
        void setReferenceFrame(tf::Pose pose);          

        /**
         * @brief Sets the type of the used control law
         * 
         * @param type Type of the controller as defined in controllerType
         */
        void setType(Controller::ControllerType type);

        /**
         * @brief Set the Control Parameter object
         * 
         * @param param Lyapunov parameters
         */
        void setControlParameter(Controller::LyapunovParameter param);

        /**
         * @brief Set the Control Parameter object
         * 
         * @param param Angle Distance Parameters
         */
        void setControlParameter(Controller::AngleDistanceParameter param);


        /**
         * @brief Loading of ROS parameterset for this controller
         * 
         */
        void load();
     

        

        
        /*Linking topics #################################################################################################################################
        ##################################################################################################################################################*/
        
        /**
         * @brief Links the current odometry topic to the controller. At this topic the current odomotry of the robot must be published
         * 
         * @param topic_name Name of the topic
         */
        void linkCurrentOdom(std::string topic_name);  
       
         /**
         * @brief Links the target/desired odometry topic to the controller. At this topic the desired odometry is published.
         * 
         * @param topic_name Name of the topic
         */                           
        void linkTargetOdom(std::string topic_name);

        /**
         * @brief Links the output velocity of the controller to a given topic
         * 
         * @param topic_name Name of the topic
         */
        void linkOutputVelocity(std::string topic_name);      
        
        /**
         * @brief Links the meta data of the controller to a given topic
         * 
         * @param topic_name Name of the topic
         */
        void linkOutputControlData(std::string topic_name);
        
        /*Callbacks########################################################################################################################################
        ##################################################################################################################################################*/
        
        /**
         * @brief Callback for incoming odometry data for the current robot odometry.
         * 
         * @param msg Incoming message
         */
        virtual void currentOdomCallback(nav_msgs::Odometry msg);
       
        /**
         * @brief Callback for incoming state data for the target odometry of the robot. This is overloaded by child classes.
         * 
         * @param msg Incoming message
         */                
        virtual void targetOdomCallback(nav_msgs::Odometry msg);
        
         
     

        /**
         * @brief Scope of the controller that is called in control frequence 
         * 
         */
        void execute(const ros::TimerEvent &ev);
        
        /**
         * @brief Resetting controller to initial state
         * 
         */
        void reset();   

        
        /*Service routines ####################################################################################################################
        ##################################################################################################################################################*/
       
        /**
         * @brief Service procedure for resetting the Controller
         * 
         * @param req Reqest for the service
         * @param res Responce of the service
         * @return true succeeded
         * @return false not succeeded
         */
        bool srvReset(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res);

        /**
         * @brief Service for setting the initial Pose of the controller
         * 
         * @param req Reqest that contains the pose
         * @param res Response that contains the succeded flag
         * @return true Succeeded
         * @return false Not Succeeded
         */
        bool srvSetReferenceFrame(multi_robot_msgs::SetInitialPoseRequest &req,multi_robot_msgs::SetInitialPoseResponse &res);

        /**
         * @brief Service for setting the control laws parameters
         * 
         * @param req Request that contains the parameters
         * @param res 
         * @return true Succeded
         * @return false Not Succeded
         */
        bool srvSetParameter(multi_robot_msgs::SetParameterRequest &req,multi_robot_msgs::SetParameterResponse &res);

        //###################################################################################################################################
        //###################################################################################################################################

        /**
         * @brief Converts ControlState struct to a COntrol state message
         * 
         * @param state Control state to be converted
         * @param msg Referenze to the msg the conversion is stored in
         */
        void controlState2controlStateMsg(ControlState &state,multi_robot_msgs::ControlState &msg); 


        /**
         * @brief Converts ControlDifference struct to a COntrol state message
         * 
         * @param difference ControlDifference to be converted
         * @param msg Referenze to the msg the conversion is stored in
         */
        void controlDifference2controlDifferenceMsg(ControlDifference &difference,multi_robot_msgs::ControlDifference &msg);  
        

        /**
         * @brief Converts ControlVector struct to a COntrol state message
         * 
         * @param control ControlVector to be converted
         * @param msg Referenze to the msg the conversion is stored in
         */
        void controlVector2controlVectorMsg(ControlVector &control,multi_robot_msgs::ControlVector &msg);         

        
    protected:
        ros::NodeHandle nh;                                         ///<Node Handle
        ros::NodeHandle robot_nh_;      ///< Nodehandle of the robot
        ros::NodeHandle controller_nh;  ///< Nodehandle of the controller

        tf::TransformListener* listener;                            ///<Listener for any transformation
        tf::TransformBroadcaster broadcaster_;                      ///<Broadcaster for broadcasting transformations
       
        std::string world_frame;                                    ///<Name of the world frame
        std::string name;                                            ///<Name of the node respective Controller
        
        ControlState current_state_;                                ///<The current state of the robot
        ControlState target_state_;                                 ///<The target state of the robot
        ControlState target_state_old_;                             ///<Old target state
        tf::Vector3 vel_old;                                         ///<The target state of a timepstep backwards for numerical differentiaiton
        tf::Vector3 acc_;                                            ///<Actual acceleration
        Buffer<tf::Vector3> buffer_;                                ///<Buffer object for using filters
        std::vector<double> time_buffer_;                           ///<Buffer for time inputs
        std::vector<tf::Vector3> velocity_buffer_;                  ///<Buffer for Velocity inputs
        double time_old_;                                        ///<Time of the old state


        ControlDifference control_dif_;                             ///<Transformation from current configuration to target configuration
        ControlVector control_;                                     ///<The calculated control vector

        tf::Transform reference_frame_;                             ///<Transformation from a world to the controllers refrence frame
        ControllerType type;                                         ///<Type of control algorythm that is used
        VelocityCartesian thresh_; ///<Threshold velocity. Values smaller then thresh are interpreted as zero
    private:
        ros::Publisher pub_cmd_vel;                                  ///<publisher object for velocity outoput topic
        ros::Publisher pub_state_out;                                ///<publisher object for state output topic
        ros::Publisher pub_control_data;                             ///<publisher object for control difference topic
      
        ros::Subscriber sub_odom_current;                            ///<Subscriber object for odometry
        ros::Subscriber sub_odom_target;                             ///<Subscriber object for target odometry topic

        ros::ServiceServer srv_reset;                                ///<Service for resetting the controller
        ros::ServiceServer srv_set_reference_frame_;     ///<Service for setting the initial pose
        ros::ServiceServer srv_set_parameter_;      ///<Service for setting the control parameters
        ros::Timer time_scope_;                                      ///<Timer for control scope

        bool publish_tf_;   ///<Flag if baselink should be published by the controller

       
      
        LyapunovParameter lyapunov_parameter_;                        ///<Parameter set for lyapunov determinations
        AngleDistanceParameter angle_distance_parameter_;   ///<Parameter set for angle distance control law
       


        void publish();                                              ///<Publish all outgoing data

        /**
         * @brief Adding the world frame. Broadcastes a world frame respectiveliy the robot reference frame expressed in world coordinates.
         * 
         */
        void publishReferenceFrame();
        /**
         * @brief Method for publishing the metadata of the conroller (current, target and difference states)
         * 
         */
        void publishControlMetaData();

        /**
         * @brief Method for publishing the cmd_vel
         * 
         */
        void publishVelocityCommand();

        /**
         * @brief Method for publishing the baselink
         * 
         */
        void publishBaseLink();


        /*Calculations and executions ####################################################################################################################
        ##################################################################################################################################################*/
       
       /**
        * @brief Algorithm for use with lyapunov based control law
        * 
        * @param parameter Parameters of the controllaw
        * @param target   Target state that should be reached by control
        * @param current    Current state of the robot
        * @return ControlVector Conrol command calculated by the control law
        */
        ControlVector calcLyapunov(LyapunovParameter parameter,ControlState target, ControlState current);
 
        /**
         * @brief Law for calculate the control vector in use with the angle distance control law
         * 
         * @param parameter Parameters of the control law
         * @param target    Target state that should be reached by control 
         * @param current   Current state of the robot
         * @return ControlVector ControlVector Conrol command calculated by the control law
         */
        virtual ControlVector calcAngleDistance(AngleDistanceParameter parameter,ControlState target, ControlState current);

        /**
         * @brief Calculates the feed-forward of velocities
         * 
         * @return ControlVector Control Vector for the robot
         */
        virtual ControlVector calcOptimalControl();

        /**
         * @brief Pass input velocity to output
         * 
         * @return ControlVector Input velocity given to output
         */
        ControlVector passVelocity();

        
};