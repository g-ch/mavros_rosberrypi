#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <mavros_extras/Attitude.h>

//Don't forget to add codes in mavros_plugins.xml
namespace mavplugin {
class AttitudePlugin : public MavRosPlugin{

public:
    AttitudePlugin():
        attitude_plugin_nh("~attitude_plugin"),
        uas(nullptr)
    { };

    void initialize(UAS &uas_)
    {
        uas = &uas_;

        // general params
        attitude_plugin_nh.param<std::string>("frame_id", frame_id, "attitude_plugin");
        attitude_plugin_pub = attitude_plugin_nh.advertise<mavros_extras::Attitude>("attitude", 1000); //add publisher to handler
    }

    const message_map get_rx_handlers() {
        return {
            MESSAGE_HANDLER(MAVLINK_MSG_ID_ATTITUDE, &AttitudePlugin::handle_attitude_msg)
        };
    }

private:
    ros::NodeHandle attitude_plugin_nh;
    ros::Publisher attitude_plugin_pub;
    UAS *uas; 
    std::string frame_id;

    void handle_attitude_msg(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
        mavlink_attitude_t attitude_msg;
        mavlink_msg_attitude_decode(msg, &attitude_msg);//decode 
        //auto header = uas->synchronized_header(frame_id,clarence_msg.mavros_a);//add  header,XXXX

        auto attitude_mav_msg = boost::make_shared<mavros_extras::Attitude>();//define a msg the same type as  mavros_extras::ClarenceNewMavros
        attitude_mav_msg->roll = attitude_msg.roll; 
        attitude_mav_msg->pitch = attitude_msg.pitch; 
        attitude_mav_msg->yaw = attitude_msg.yaw; 
        attitude_mav_msg->rollspeed = attitude_msg.rollspeed; 
        attitude_mav_msg->pitchspeed = attitude_msg.pitchspeed; 
        attitude_mav_msg->yawspeed = attitude_msg.yawspeed; 

        attitude_plugin_pub.publish(attitude_mav_msg); //publish msg
    }
};
};
PLUGINLIB_EXPORT_CLASS(mavplugin::AttitudePlugin, mavplugin::MavRosPlugin)
