#ifndef ANIMATION_HOST_H
#define ANIMATION_HOST_H

#include <tf/tf.h>

#include <qt5/QtCore/QtCore>

#include "utils.h"
#include "olamanager.h"
#include "pixelmapper.h"

struct BlobInfo {
    tf::Vector3 centroid;
    tf::Vector3 bounds;
    tf::Vector3 realDimensions;
    ros::Time timestamp;
};

struct RenderData {
    QMultiMap<int, BlobInfo*> blobs;
    ros::Time timestamp;
};

class Animation {
    public:
        virtual void renderFrame(QImage* image, const RenderData& data) = 0;
};

class BlobTracker {
    public:
        BlobTracker(QSharedPointer<RenderData> data);

        void insertBlob(BlobInfo* b);
        void setMaxAgeMs(quint64 ms);
        void setMaxJoinRadius(qreal radius);
        void updateBlobs(QList<BlobInfo*> blobs);

    private:
        QSharedPointer<RenderData> _dataPtr;
        quint64 _maxAgeMs;
        tfScalar _maxJoinRadius;
};

/*
class Effect : public Animation {
    public:
        virtual void renderFrame(QImage* image, const RenderData& data);

    private:
        RenderData _startInfo;
        QMap<int, Animation*> _animations;
};

class EffectEngine {
    public:

    private:
        //QList

};*/

class AnimationHost {
    public:
        AnimationHost(QString pixelMapPath, QSharedPointer<RenderData> data);
        ~AnimationHost();

        PixelMapper* getPixelMapper() const;

        Animation* insertLayer(int layer, Animation* a);
        void removeLayer(int layer);
        Animation* getLayer(int layer);

        void transmit();
        QImage* renderAll();
        QImage* renderLayer(int minLayer, int maxLayer=0);

    private:
        OlaManager* _olaManager;
        PixelMapper* _pixelMapper;

        //EffectEngine* _effectEngine;

        QMap<int, Animation*> _animations;
        QSharedPointer<RenderData> _dataPtr;
        int _frameCount;
};

#endif  //ANIMATION_HOST_H
