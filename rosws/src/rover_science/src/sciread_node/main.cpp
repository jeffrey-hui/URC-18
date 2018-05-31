//
// Created by matthew on 5/31/18.
//

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <simpli2c/simpli2c.h>

namespace rover_science {

    const int SCIREAD_BUS = 1, SCIREAD_ADDRESS = 0x11;

    template<int dCount>
    class SciRead {
    public:
        SciRead()  : dev(SCIREAD_BUS, SCIREAD_ADDRESS) {
            memset(arr, 0, sizeof(float) * dCount);
        };
        ~SciRead() {
            dev.close_();
        }

        void updateArray() {
            dev.requestMany(0x01, sizeof(float) * dCount, reinterpret_cast<uint8_t *>(arr));
        }

        float arr[dCount];
    private:
        simpli2c::Device dev;
    };

}

int main(int argc, char ** argv) {
    ros::init(argc, argv, "sciread_node");

    ros::NodeHandle nh;
    rover_science::SciRead<8> device;
    ROS_INFO_STREAM("Starting sciread_node");

    ros::Publisher pub = nh.advertise<std_msgs::Float32MultiArray>("/science/data", 10);
    ros::Rate r = ros::Rate(0.6);

    std_msgs::Float32MultiArray m;
    m.data.resize(8);
    while (nh.ok()) {
        r.sleep();
        device.updateArray();
        m.data.clear();
        m.data.assign(device.arr, std::end(device.arr));
        pub.publish(m);
    }

    return 0;
}