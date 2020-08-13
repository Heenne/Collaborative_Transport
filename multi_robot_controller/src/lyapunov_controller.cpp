#include<multi_robot_controller/lyapunov_controller.h>

LyapunovController::LyapunovController(ros::NodeHandle &nh,LyapunovParameter parameterset):Controller(nh),parameter_(parameterset)
{
    
}
LyapunovController::LyapunovController(ros::NodeHandle &nh):Controller(nh)
{
    ros::NodeHandle priv("~");
    std::vector<float> lyapunov;
    if(!priv.getParam("lyapunov",lyapunov))
    {
        throw NecessaryParamException(priv.resolveName("lyapunov"));
    }
    else
    {
        this->parameter_=LyapunovParameter(lyapunov);
    }          

}

Controller::ControlVector LyapunovController::calcControl(State current_state ,State target_state)
{
    double omega=target_state.ang_vel.z();
    double v=sqrt(pow(target_state.lin_vel.x(),2)+pow(target_state.lin_vel.y(),2)); 
    
    tf::Transform control_dif_=current_state.pose.inverseTimes(target_state.pose);
    
    double x=control_dif_.getOrigin().getX();
    double y=control_dif_.getOrigin().getY();
    double phi=tf::getYaw(control_dif_.getRotation());


    ControlVector output;
    output.v=this->parameter_.kx*x+v*cos(phi);
    output.omega=omega+this->parameter_.ky*v*y+this->parameter_.kphi*sin(phi);

    return output;      
}