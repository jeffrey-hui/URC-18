//
// Created by matthew on 5/30/18.
//

#include "oxyarm_dev.h"

rover_science::OxyArm::OxyArm() : dev(OXY_ARM_BUS, OXY_ARM_ADDRESS) {
    dev.open_();
    upd.add("oxy_arm", this, &OxyArm::produce_diagnostics);
}

rover_science::OxyArm::~OxyArm() {
    dev.close_();
}

void rover_science::OxyArm::writeMotor(int16_t val) {
    char buf[3] = {
            0x02,
            0x00,
            0x00
    };
    memcpy(buf + 1, &val, 2);
    this->dev.writeMany(reinterpret_cast<uint8_t *>(buf), 3);
    this->upd.update();
}

float rover_science::OxyArm::readOxy() {
    float buf;
    this->dev.requestMany(0x01, sizeof(float), reinterpret_cast<uint8_t *>(&buf));
    this->upd.update();
    return buf;
}

void rover_science::OxyArm::produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat) {
    if (this->dev.isOpen()) {
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Connected to oxy-arm ultra strength");
        stat.add("address", OXY_ARM_ADDRESS);
    }
    else {
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Not connected to oxy-arm ultra strength");
        stat.add("address", OXY_ARM_ADDRESS);
    }
}

