//
// Created by matthew on 5/15/18.
//

#include <ros/ros.h>
#include <simpli2c/simpli2c.h>
#include <nmea_msgs/Sentence.h>

const int ADDRESS = 0x48;
const int BUS = 1;

int main(int argc, char ** argv) {
    ros::init(argc, argv, "gps_forwarder");
    ros::NodeHandle nh("~");
    ros::NodeHandle ng("/");

    ros::Publisher pub = ng.advertise<nmea_msgs::Sentence>("/gps/fix", 10, true);

    simpli2c::Device dev(BUS, ADDRESS);
    ros::Duration d(0.1);
    while (ros::ok()) {
        ros::spinOnce();
        uint8_t statusCode = dev.readOne();
        if (statusCode == 0x00) {
            d.sleep();
        }
        else {
            uint8_t msgLength = dev.readOne();
            std::string result;
            char buf[msgLength];
            dev.readMany(msgLength, reinterpret_cast<uint8_t *>(buf));
            result.assign(buf, msgLength);

            nmea_msgs::Sentence sentence;
            sentence.sentence = result;
            pub.publish(sentence);
        }
    }

    ros::shutdown();

    // this has to take simpli2c and output nmea_msgs/Sentence messages.
    return 0;
}
