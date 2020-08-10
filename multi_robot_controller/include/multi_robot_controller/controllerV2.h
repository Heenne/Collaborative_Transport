#include<multi_robot_controller/input_pose_odom.h>
#include<multi_robot_controller/constrained_rigid_motion.h>
#include<tf/tf.h>
class Controller
{
private:
    InputPoseOdom input_handler_;
    ConstrainedRigidMotion rigid_motion_;
    struct State
    {

    };
    void controlScope();
protected:
    virtual void calcControl(State target_state,State current_state)=0;
public:
    Controller(ros::NodeHandle nh);
    ~Controller();    
    
};

