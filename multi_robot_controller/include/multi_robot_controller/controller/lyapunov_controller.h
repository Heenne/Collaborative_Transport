#include<multi_robot_controller/controller/controller.h>
#include<multi_robot_controller/LyapunovConfig.h>
#include<dynamic_reconfigure/server.h>

/**
 * @brief Class that implements a lyapunov approach based controller
 * 
 * |Ros-Parameter   |Description        |
 * |----------------|-------------------|
 * |~lyapunov | Array of parameters it the order of LyapunovParameter (kx,ky,kphi)|
 * 
 */

class LyapunovController:public Controller
{
    public:
        /**
         * @brief Holds the necessary parameters for the control law.
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
            LyapunovParameter(float kx=0.0,float ky=0.0,float kphi=0.0)
            {this->kx=kx;this->ky=ky;this->kphi=kphi;}

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
         * @brief Construct a new Lyapunov Controller object
         * 
         * @param nh Nodehandle the controller is determining topic namespaces with
         * @param parameterset Set of LyapunovParameter for calculations
         */
        LyapunovController(ros::NodeHandle &nh,LyapunovParameter parameterset);
        /**
         * @brief Construct a new Lyapunov Controller object and automatically load parameters from the ros
         *          parameter server
         * 
         * @param nh Nodehandle the controller is determining topic namespaces with
         */
        LyapunovController(ros::NodeHandle &nh);
        

    private:
        LyapunovParameter parameter_; ///< Prarameter set of the controller (LyapunovParameter)
        dynamic_reconfigure::Server<multi_robot_controller::LyapunovConfig> server_;  ///<Server for the dynamic reconfiguration of the parameters
        ControlVector calcControl(State current_state ,State target_state) override;  ///<Ovverrides the ControlLaw from base class with the lyapunov based control law
        void dynConfigcallback(multi_robot_controller::LyapunovConfig &config, uint32_t level); ///<Callback for the dynamic reconfigure server
        
};