#include "arduino_node.h"

Servo s;
double p = 0;
double o = 0;
int d = 0;

ros::NodeHandle rosNode;

void setArmPos(const std_msgs::Float64MultiArray& arm_pos) { servoWrite(arm_pos.data[0]); }
ros::Subscriber<std_msgs::Float64MultiArray> armPosSub("arm_pos", setArmPos);

void setArmSpeed(const std_msgs::Float64MultiArray& arm_speed) {
    o = (double)arm_speed.data[0];
    d = (abs(o) < 0.001) ? 0 : ((int)(1000.0 * M_PI / (180.0 * o)));
}
ros::Subscriber<std_msgs::Float64MultiArray> armSpeedSub("arm_speed", setArmSpeed);

std_msgs::Float64MultiArray posMultiArray;
ros::Publisher posPub("arm_pos_feedback", &posMultiArray);

void servoWrite(float d) {
    p = constrain(d, 0, 180);
    s.write(p);
    posMultiArray.data[0] = p;
    posPub.publish(&posMultiArray);
}

void setup() {
    float positions[AXES] = {0.0f};
    posMultiArray.data = positions;
    posMultiArray.data_length = AXES;
    rosNode.initNode();
    rosNode.subscribe(armPosSub);
    rosNode.subscribe(armSpeedSub);
    s.attach(3, 600, 2400);
    servoWrite(0);
    rosNode.advertise(posPub);
}

void loop() {
    rosNode.spinOnce();
    if (d != 0) {
        servoWrite(p + sgn(d));
        delay(abs(d));
        if ((p == 0) || (p == 180)) {
            d = 0;
            o = 0;
        }
    } else {
        delay(1);
    }
}

inline int sgn(double x) {
    if (x == 0) return 0;
    return (x > 0) ? 1 : (-1);
}