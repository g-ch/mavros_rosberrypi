#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <mavros_extras/LazerDistance.h>
#include "std_msgs/Float32.h" 
#include <ros/console.h>

namespace mavplugin{

class LazerDistancePlugin : public MavRosPlugin{

public:
	 LazerDistancePlugin():
	 lazer_distance_nh("~lazer_distance"),
	 uas(nullptr)
    { };

    void initialize(UAS &uas_){
    	uas = &uas_;
        
    	lazer_distance_nh.param<std::string>("frame_id", frame_id, "lazer_distance");
        //subcribe the topic and excute the callback function
    	lazer_distance_sub = lazer_distance_nh.subscribe("/chatter",500,&LazerDistancePlugin::lazer_distance_send_cb,this);

    }
    
    std::string get_name() {
		return "lazer_distance";
	}


     const message_map get_rx_handlers() {
		return {
			     
		};
	}


private:
	ros::NodeHandle lazer_distance_nh;
	ros::Subscriber lazer_distance_sub;
	UAS *uas;

	std::string frame_id;

    void lazer_distance_send(float a, float b, float c, float d){
    	mavlink_message_t lazer_distance_msg;

    	mavlink_msg_lazer_distance_pack_chan(UAS_PACK_CHAN(uas),&lazer_distance_msg,a,b,c,d); //pack
    	UAS_FCU(uas)->send_message(&lazer_distance_msg); //send
        
        ROS_INFO("float_a %d %d", lazer_distance_msg.seq,lazer_distance_msg.len);
    	
    }
    
    //callbacks
    void lazer_distance_send_cb(const std_msgs::Float32 &msg){
        lazer_distance_send(msg.data,4.0,3.0,0.1);
    }
};

};

PLUGINLIB_EXPORT_CLASS(mavplugin::LazerDistancePlugin, mavplugin::MavRosPlugin)
