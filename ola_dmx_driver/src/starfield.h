#ifndef STARFIELD_H
#define STARFIELD_H

#include <ros/ros.h>
#include <tf/tf.h>

#include <qt5/QtCore/QtCore>
#define G (-9.8070f)

using namespace std;

struct StarInfo {
    StarInfo();
    void updatePosition();

    tf::Vector3 position;
    tf::Vector3 velocity;
    tf::Vector3 force;
    tfScalar mass;
    ros::Time created;
    tfScalar maxDuration;
};

class StarfieldState {
    public:

        QList<StarInfo> wells;     //Non moving gravity wells
        QList<StarInfo> objects;   //Moving objects

        //static const tfScalar G = -9.81;

        StarfieldState();

        /**
         * @brief update
         *          -Remove old objects
         *          -Compute object forces
         *          -Update object physics {position, velocity, momentum}
         *
         */
        void update();


};

#endif  //STARFIELD_H

