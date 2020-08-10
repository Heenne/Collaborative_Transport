#include<eigen3/Eigen/Dense>
#include<eigen3/Eigen/Geometry>
#include<vector>

class ConstrainedRigidMotion{

    public:
        ConstrainedRigidMotion();
        ConstrainedRigidMotion(Eigen::Vector3d reference);
        void updateInputState(Eigen::Vector3d state,Eigen::Vector3d d_state,double time);
        Eigen::Vector3d getDiffState();
        Eigen::Vector3d getState();

        static Eigen::Matrix3d createDiffDriveLocking();
    private:
        void calcConstrains(); 
        void calcUnConstrained();     
        void applyConstrains();

        Eigen::Quaterniond rotation_;
        Eigen::Matrix3d angular_tensor_;
        Eigen::Vector3d state_out_;
        Eigen::Vector3d state_in_;
        Eigen::Vector3d d_state_out_;
        Eigen::Vector3d d_state_in_;
        Eigen::Vector3d reference_;
        Eigen::Matrix3d locking_;
        Eigen::Vector3d constrain_;
        Eigen::Vector3d d_constrain_;
        double time_old_;
        double time_new_;
        bool initial_call_;
      

};