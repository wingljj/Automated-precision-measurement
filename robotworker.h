#ifndef ROBOTWORKER_H
#define ROBOTWORKER_H

#include <QObject>
#include <QVector>
#include <QAtomicInt>
#include "irobodk.h"
#include "iitem.h"

class RobotWorker : public QObject
{
    Q_OBJECT

public:
    explicit RobotWorker(RoboDK* rdk, QObject* parent = nullptr);
    ~RobotWorker();

signals:
    void moveStarted();
    void moveCompleted(bool success, QString message);

public slots:
    void executeMoveCartesian(Item robot, QVector<double> xyzwpr, int mode);
    void executeMoveJoints(Item robot, QVector<double> joints);
    void executeSetSpeed(Item robot, double speed_linear, double speed_joints, double accel_linear, double accel_joints);
    void setStopRequested(bool stop);
    void stopMovement();

private:
    RoboDK* m_rdk;
    QAtomicInt m_stopFlag;
};

#endif // ROBOTWORKER_H
