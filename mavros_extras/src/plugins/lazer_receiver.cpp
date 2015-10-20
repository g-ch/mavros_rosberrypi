#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <mavros_extras/LazerDistance.h>
#include "std_msgs/Float32.h" 

//Don't forget to add codes in mavros_plugins.xml
namespace mavplugin {
class LazerReceiverPlugin : public MavRosPlugin{

public:
        LazerReceiverPlugin():
                lazer_receiver_nh("~lazer_receiver"),
                uas(nullptr)
                
        { };

        void initialize(UAS &uas_)
	{
		uas = &uas_;

		// general params
		lazer_receiver_nh.param<std::string>("frame_id", frame_id, "lazer_receiver");
        lazer_receiver_pub = lazer_receiver_nh.advertise<mavros_extras::LazerDistance>("lazer_receiver", 1000); //add publisher to handler
	}

	const message_map get_rx_handlers() {
		return {
			      MESSAGE_HANDLER(MAVLINK_MSG_ID_LAZER_DISTANCE, &LazerReceiverPlugin::handle_lazer_distance)
		};
	}

private:
        ros::NodeHandle lazer_receiver_nh;
        ros::Publisher lazer_receiver_pub;
        UAS *uas; 
        std::string frame_id;
        

        void handle_lazer_distance(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_lazer_distance_t lazer_msg;
		mavlink_msg_lazer_distance_decode(msg, &lazer_msg);//decode 
                
                //auto header = uas->synchronized_header(frame_id,clarence_msg.mavros_a);//add  header,XXXX

                auto lazer_mav_msg = boost::make_shared<mavros_extras::LazerDistance>();//define a msg the same type as  mavros_extras::ClarenceNewMavros

                lazer_mav_msg->min_distance = lazer_msg.min_distance; 
                lazer_mav_msg->angle = lazer_msg.angle; 
                lazer_mav_msg->lazer_x = lazer_msg.lazer_x; 
                lazer_mav_msg->lazer_y = lazer_msg.lazer_y; 

                
                lazer_receiver_pub.publish(lazer_mav_msg); //publish msg

               }
       
            
};
};
PLUGINLIB_EXPORT_CLASS(mavplugin::LazerReceiverPlugin, mavplugin::MavRosPlugin)
