#ifndef ORIENTATION_FEED_FORWARD_H
#define ORIENTATION_FEED_FORWARD_H

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>


/** @addtogroup FeedForward
 * @{
 */
/**
 * @brief A Orientation feed forward class. It uses a given orientation and calculates a robot arm pose
 * that compansates the motion by the change of orientation (null space motion). 
 * 
 */
class OrientationFeedForward{
    public:
        typedef Eigen::Matrix<double,3,1> Position;     //<Typedef for Position vectors
        typedef Eigen::Quaterniond Orientation;         //<Typedef for Orientation quaternions
        typedef Eigen::Matrix<double,7,1> Pose;         //<Typedef for Pose Vectors (combination of position and quaternion)

        /**
         * @brief Construct a new Orientation Feed Forward object
         * 
         */
        OrientationFeedForward();
        /**
         * @brief Construct a new Orientation Feed Forward object
         * 
         * @param ori_off The Orientation offset the Robot base has with respect to the mobile base frame
         * @param pos_offf The Position offset the Robot base has with respect to the mobile base frame
         */
        OrientationFeedForward(Position pos_offf,Orientation ori_off);
       
       /**
        * @brief Updates the current Orientation of the mobile base that has to be feed forwarded
        * 
        * @param angle Angle of the Rotation around z-axis
        */
        void updateOrientation(double angle);
        /**
         * @brief Updates the current Orientation of the mobile base that has to be feed forwarded
         * 
         * @param ori Quaternion of the current Orientation
         */
        void updateOrientation(Orientation ori);
        
        /**
         * @brief Get the Pose object for motion control
         * 
         * @return Pose in robot base frame wich can be forwarded to the robot motion control
         */
        Pose getPose();

        /**
         * @brief Set the Offset object
         * 
         * @param ori_off Orientation of the Offset (from the mobile base frame and the robot frame)
         * @param pos_off Position of the Offset (from mobile base frame and the robot frame)
         */
        void setOffset(Position pos_off,Orientation ori_off);
        
        /**
         * @brief Set the Offset object
         * 
         * @param ori_off Orientation of the Offset (from the mobile base frame and the robot frame)
         * @param pos_off Position of the Offset (from mobile base frame and the robot frame)
         */
        void setOffset(Pose pose);

        /**
         * @brief Set the Desired Pose that has to be hold in mobile base frame
         * 
         * @param pose Desired Pose with in mobile base frame 
         */
        void setDesiredPose(Pose pose);
        /**
         * @brief Set the Desired Pose object
         * 
         * @param position Position that has to be hold by the endeffektor defined in mobile base frame
         * @param orientation Orientation that has to be hold by the endeffektor defined in robot base frame
         */
        void setDesiredPose(Position position,Orientation orientation) ; 
        /**
         * @brief Set the Initial orientation to be respected
         * 
         * @param initial_ori orientation given at initialisation time
         */
        void setInitial(Orientation initial_ori);
    protected:
       
    private:
        Position transformPosition();
        Orientation transformOrientation();
        
        Orientation current_ori_;
        Orientation initial_ori_;
        
        Position d_pos_fixed_;
        Orientation d_ori_fixed_;
        Eigen::Transform<double,3,Eigen::Affine> offset_;
        Eigen::Transform<double,3,Eigen::Affine> desired_ee_;
        Eigen::Transform<double,3,Eigen::Affine> current_;

        Orientation offset_rotation_;
        Position offset_position_;       
};
//@}
#endif