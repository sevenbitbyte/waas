#ifndef STARFIELD_H
#define STARFIELD_H

#include <ros/ros.h>
#include <tf/tf.h>

#include <qt5/QtCore/QtCore>

#include "animations.h"

using namespace std;

struct StarInfo {
    enum PositionMethod { Static=0, Physics=3, Sensor=2 };
    enum ObjectType { Star=1, Emitter=2, Attractor=3, Repulsor=4};

    StarInfo(ObjectType t=Star, PositionMethod m = Physics);
    StarInfo(int id, BlobInfo* blob, ObjectType t=Star, PositionMethod m=Sensor);
    void updatePosition();

    tf::Vector3 position;
    tf::Vector3 velocity;
    tf::Vector3 force;
    tfScalar mass;
    ros::Time created;
    ros::Duration maxDuration;
    tfScalar radius;	// Different Meaning based upon type
    ObjectType type;
    PositionMethod method;
    int trackedBlobId;

    bool operator==(const StarInfo& other);
};

class Starfield {
    public:

        Starfield();

        /**
         * @brief update
         *          -Remove old objects
         *          -Compute object forces
         *          -Update object physics {position, velocity, momentum}
         *
         */
        void update(const RenderData& blobs);

        bool isTracked(int blobId);
        void insertStar(StarInfo* s);
        void removeStar(StarInfo* s);
        void removeStars(QList<StarInfo*> list);

        void setBounds(QRectF bounds);
        QList<StarInfo*> getStars() const;

    private:
        tfScalar _gravity;
        int _maxStars;
        int _minStars;
        int _starCount;
        tfScalar _emitProbability;
        QList<StarInfo*> _stars;
        QMultiMap<StarInfo::PositionMethod, StarInfo*> _starsByMethod;
        QMultiMap<StarInfo::ObjectType, StarInfo*> _starsByType;
        QMultiMap<int, StarInfo*> _starsByBlobId;
        QRectF _bounds;
};

class StarSim : public Animation {
    public:
        StarSim();
        virtual void renderFrame(QImage* image, const RenderData& data);

        Starfield state;
};

#endif  //STARFIELD_H

