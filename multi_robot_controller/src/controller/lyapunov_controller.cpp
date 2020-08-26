#include<multi_robot_controller/controller/lyapunov_controller.h>

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
    
    this->server_.setCallback(boost::bind(&LyapunovController::dynConfigcallback,this,_1,_2));     

}

Controller::ControlVector LyapunovController::calcControl(State current_state ,State target_state)
{
    double omega=target_state.ang_vel.z();
    double v=sqrt(pow(target_state.lin_vel.x(),2)+pow(target_state.lin_vel.y(),2)); 
    
    tf::Transform control_dif=current_state.pose.inverseTimes(target_state.pose);
    
    double x=control_dif.getOrigin().getX();
    double y=control_dif.getOrigin().getY();    
  
    double phi=tf::getYaw(control_dif.getRotation());
    ROS_INFO_STREAM("Raw: "<<phi);
    if(phi>=M_PI_2)
    {
        phi-=M_PI;
        v=-v;
        ROS_INFO_STREAM("Quadrant 2: "<<phi);       
    }
    else if( phi<=-M_PI_2)
    {
        
        phi=phi+M_PI;
        v=-v;
        ROS_INFO_STREAM("Quadrant 3: "<<phi);
        
    }    
   
    control_dif.setRotation(tf::createQuaternionFromYaw(phi));
    this->control_diff_.pose=control_dif;
    this->control_diff_.lin_vel=target_state.lin_vel-current_state.lin_vel;
    this->control_diff_.ang_vel=target_state.lin_vel-current_state.lin_vel;

    ControlVector output;
    output.v=this->parameter_.kx*x+v*cos(phi);
    output.omega=omega+this->parameter_.ky*v*y+this->parameter_.kphi*sin(phi);

    return output;      
}

void LyapunovController::dynConfigcallback(multi_robot_controller::LyapunovConfig &config, uint32_t level) {
    ROS_INFO("Reconfigure Request: %lf %lf %lf",config.kx,config.ky,config.kphi);
    this->parameter_=LyapunovParameter(config.kx,config.ky,config.kphi);
}