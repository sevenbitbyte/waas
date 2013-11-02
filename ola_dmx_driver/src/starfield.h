#ifndef STARFIELD_H
#define STARFIELD_H

#include "ros/ros.h"
#include "tf/tf.h"

#include <vector>

using namespace std;

struct StarfieldState {
    struct StarInfo {
        StarInfo();
        void updatePosition();

        tf::Vector3 position;
        tf::Vector3 velocity;
        tf::Vector3 force;
        tf::tfScalar mass;
        ros::Time created;
        tfScalar maxDuration;
    };

    vector<StarInfo> wells;     //Non moving gravity wells
    vector<StarInfo> objects;   //Moving objects

    static const tfScalar G = -9.81;

    StarfieldState();

    /**
     * @brief update
     *          -Remove old objects
     *          -Compute object forces
     *          -Update object physics {position, velocity, momentum}
     *
     */
    void update();

    //tf::Vector3 computeGravitationalForce(tf::Vector3 position);

};

#endif  //STARFIELD_H

