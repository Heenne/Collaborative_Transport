#include<eigen3/Eigen/Dense>
#include<eigen3/Eigen/Geometry>
#include<vector>

/** \addtogroup RigidMotion 
 *  @{
 */

class Constrainedrigid_motion{

    public:
        /**
         * @brief Construct a new Constrainedrigid_motion object
         * 
         */
        Constrainedrigid_motion();
        /**
         * @brief Construct a new Constrainedrigid_motion object with given refrence vector
         * 
         * @param reference 3D Vector that describes the x,y,phi offset from point to transformed point
         */
        Constrainedrigid_motion(Eigen::Vector3d reference);
        /**
         * @brief Update procedure for the motion states
         * 
         * @param state State representation of the source
         * @param d_state Velocities (state derivatives) of the  source
         * @param time Timestamp, the update was procceded at (important for numerical derivatives)
         */
        void updateInputState(Eigen::Vector3d state,Eigen::Vector3d d_state,double time);
        /**
         * @brief Get the Velocities of the destination
         * 
         * @return Eigen::Vector3d Velocities (state derivatives) of the destination
         */
        Eigen::Vector3d getDiffState();
        /**
         * @brief Get the State of destination
         * 
         * @return Eigen::Vector3d State vector of the destination
         */
        Eigen::Vector3d getState();
        /**
         * @brief Set the reference from source to destination
         * 
         * @param ref x,y,phi representation of the reference
         */
        void setReference(Eigen::Vector3d ref);
        /**
         * @brief Create a differential drive locking matrix
         * 
         * @return Eigen::Matrix3d differential drive locking matrix
         */
        static Eigen::Matrix3d createDiffDriveLocking();
    private:
        void init();
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