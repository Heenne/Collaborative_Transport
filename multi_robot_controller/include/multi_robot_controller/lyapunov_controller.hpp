#include<multi_robot_controller/controllerV2.h>
class LyapunovController:public Controller
{
    public:
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

      
        inline LyapunovController(ros::NodeHandle &nh,LyapunovParameter parameterset):Controller(nh),parameter_(parameterset)
        {
           
        }
        inline LyapunovController(ros::NodeHandle &nh):Controller(nh)
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

    private:
        LyapunovParameter parameter_;
        inline ControlVector calcControl(State target_state,State current_state) override
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

};