#include<multi_robot_controller/controllerV2.h>
class PassTroughController:public Controller
{
    public:
        inline PassTroughController(ros::NodeHandle nh):Controller(nh)
        {
        }
    private:
        inline ControlVector calcControl(State target_state,State current_state) override
        {
            tf::Vector3 lin=target_state.lin_vel;
            tf::Vector3 ang=target_state.ang_vel;
            return ControlVector(std::sqrt(lin.x()*lin.x()+lin.y()*lin.y()),ang.z());            
        }

};