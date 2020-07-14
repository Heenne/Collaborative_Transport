#ifndef ORIENTATION_FEED_FORWARD_H
#define ORIENTATION_FEED_FORWARD_H

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

class OrientationFeedForward{
    public:
        typedef Eigen::Matrix<double,3,1> Position;
        typedef Eigen::Quaterniond Orientation;
        typedef Eigen::Matrix<double,7,1> Pose;

        OrientationFeedForward();
        Pose updateOrientation();
        Pose updateOrientation(double angle);
        Pose updateOrientation(Orientation ori);

        void setDesiredPose(Pose pose);  
        void registerPoseHandler();
    protected:
        Pose last_pose_;
       
    private:
        Pose d_pose_fixed_;
        Position d_pos_fixed_;
        Orientation d_ori_fixed_;
        
        
        
};
#endif