#include "animations.h"

FillFade::FillFade() {
    firstRender = ros::Time::now();
    duration = ros::Duration(5);
}

void FillFade::renderFrame(QImage* image, const RenderData& data) {
    Q_ASSERT(image != NULL);
    Q_ASSERT(image->width() > 0);
    Q_ASSERT(image->height() > 0);

    ros::Duration delta = data.timestamp - firstRender;

    double durationDelta = (double) (delta.toNSec() % duration.toNSec());
    double position = durationDelta / (double) duration.toNSec();

    QColor color = QColor::fromHsvF( qMin(1.0,position), 0.8, 0.3 );
    image->fill(color);
}


StarPath::StarPath() {
    duration = ros::Duration(1.5);
}

void StarPath::renderFrame(QImage* image, const RenderData& data) {
    Q_ASSERT(image != NULL);
    Q_ASSERT(image->width() > 0);
    Q_ASSERT(image->height() > 0);

    QPainter painter( image );

    foreach(BlobInfo* blob, data.blobs){
        ros::Duration delta = data.timestamp - blob->timestamp;

        double durationDelta = (double) (delta.toNSec() % duration.toNSec());
        qreal position = durationDelta / (double) duration.toNSec();

        if(delta > duration){
            continue;
        }

        double widthPx = blob->bounds.x();
        double depthPx = blob->bounds.y();

        double centerXPx = blob->centroid.x();
        double centerYPx = blob->centroid.y();

        double radiusPx = qMax(widthPx, depthPx);
        double radius = qMax(blob->realDimensions.x(), blob->realDimensions.y());

        QRectF bounds(centerXPx - (widthPx/2.0f),
                      centerYPx - (depthPx/2.0f),
                      radiusPx,
                      radiusPx);
        QBrush fillBrush;

        if(radius > 0.75f){
            QConicalGradient conicalGrad(centerXPx,centerYPx, position*360.0f);
            conicalGrad.setColorAt(0, Qt::red);
            conicalGrad.setColorAt(90.0/360.0, Qt::green);
            conicalGrad.setColorAt(180.0/360.0, Qt::blue);
            conicalGrad.setColorAt(270.0/360.0, Qt::magenta);
            conicalGrad.setColorAt(360.0/360.0, Qt::yellow);

            fillBrush = QBrush( conicalGrad );

        }
        else {
            QRadialGradient radialGrad(QPointF(centerXPx,centerYPx), radiusPx);
            QColor white(Qt::white);
            white.setHsvF(0, 0, 1.0-position);

            QColor pink(Qt::magenta);
            pink.setAlphaF(position);

            radialGrad.setColorAt(0, white);
            radialGrad.setColorAt(1.0f, pink);

            fillBrush = QBrush( radialGrad );
        }

        QPainterPath fillPath;

        fillPath.addEllipse(bounds);

        painter.fillPath(fillPath, fillBrush);
    }
}
