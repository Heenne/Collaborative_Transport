#ifndef TF_ORIENTATION_FEED_FORWARD_H
#define TF_ORIENTATION_FEED_FORWARD_H

#include <transport_controller/ros_orientation_feed_forward_base.h>

/** @addtogroup group_feed_forward
 * @{
 */
/**
 * @brief A Tf implementation of the OrientationFeeedForward class. It contains a tf listener that listens to a specific 
 * transformations and uses the data as input data for the feed forward control. 
 * 
*/
class TfOrientationFeedForward:public RosOrientationFeedForwardBase{
    public:
        TfOrientationFeedForward(ros::NodeHandle &nh); 
        bool init() override;     
    protected:
        void update(const ros::TimerEvent&) override;
        
    private:
        std::string current_target_frame_;
        std::string current_source_frame_;     
     
};
//@}
#endif