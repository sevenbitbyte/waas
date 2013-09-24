#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtCore>
#include <QMainWindow>

#include "utils.h"
#include "ledrun.h"
#include "olamanager.h"
#include "pixelmapper.h"

namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow
{
        Q_OBJECT
        
    public:
        explicit MainWindow(QWidget *parent = 0);
        ~MainWindow();

    public slots:
        void moveDmxCursor(bool forward);
        void animateDmxCursor();

        void cursorResetSlot();
        void cursorForwardSlot();
        void cursorBackwardSlot();

        void updateCursorAddrSlot(QString text);

        void updateUi();

        void blackoutSlot();
        void whiteoutSlot();
        void whiteoutItensitySlot(int value);

        void runStartSlot();
        void runEndSlot();

        void savePixelMap();

    private:
        Ui::MainWindow *ui;

        OlaManager* _ola;
        PixelMapper* _pixelMap;

        DmxAddress _dmxStart;
        DmxAddress _dmxEnd;

        LedRun* _nextRun;

        //Controls
        /*bool _blackoutReq;
        bool _intensityRequest;
        int _intensityValue;*/

        //Cursor variables
        QColor _cursorValue;
        QTimer* _animateTimer;
        DmxAddress _dmxCursor;
        QDateTime _lastAnimateTime;
        int _animationStage;
        int _frameCount;
        bool _forceRender;
};

#endif // MAINWINDOW_H
