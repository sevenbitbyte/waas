#ifndef ANIMATIONS_H
#define ANIMATIONS_H

#include "animationhost.h"
//#include "starfield.h"

class FillFade : public Animation {
    public:
        FillFade();

        virtual void renderFrame(QImage* image, const RenderData& data);
        ros::Time firstRender;
        ros::Duration duration;
};


class StarPath : public Animation {
    public:
        StarPath();
        virtual void renderFrame(QImage* image, const RenderData& data);

        ros::Duration duration;
};

/*
class StarSim : public Animation {
    public:
        StarSim();
        virtual void renderFrame(QImage* image, const RenderData& data);

        StarfieldState state;

        int maxObj;
};*/

#endif  //
