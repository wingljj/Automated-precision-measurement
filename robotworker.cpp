#include "robotworker.h"
#include <QDebug>
#include <QThread>

RobotWorker::RobotWorker(RoboDK* rdk, QObject* parent)
    : QObject(parent)
    , m_rdk(rdk)
    , m_stopFlag(0)
{
    if (!m_rdk) {
        qDebug() << "Warning: RobotWorker created with null RoboDK pointer";
    }
    qDebug() << "RobotWorker created in thread:" << QThread::currentThreadId();
}

RobotWorker::~RobotWorker()
{
    qDebug() << "RobotWorker destroyed";
}

void RobotWorker::stopMovement()
{
    setStopRequested(true);
}

void RobotWorker::setStopRequested(bool stop)
{
    m_stopFlag.storeRelaxed(stop ? 1 : 0);
    qDebug() << "Stop flag set to" << stop << "in thread:" << QThread::currentThreadId();
}

void RobotWorker::executeSetSpeed(Item robot, double speed_linear, double speed_joints, double accel_linear, double accel_joints)
{
    if (!robot) {
        emit moveCompleted(false, "Invalid robot object");
        return;
    }

    robot->setSpeed(speed_linear, speed_joints, accel_linear, accel_joints);
    qDebug() << "Speed set in thread:" << QThread::currentThreadId()
             << speed_linear << speed_joints << accel_linear << accel_joints;
}

void RobotWorker::executeMoveCartesian(Item robot, QVector<double> xyzwpr, int mode)
{
    qDebug() << "executeMoveCartesian in thread:" << QThread::currentThreadId();

    if (m_stopFlag.loadRelaxed() == 1) {
        emit moveCompleted(false, "Movement stopped");
        return;
    }

    m_stopFlag.storeRelaxed(0);

    if (!robot) {
        emit moveCompleted(false, "Invalid robot object");
        return;
    }

    if (xyzwpr.size() != 6) {
        emit moveCompleted(false, "Invalid parameters");
        return;
    }

    if (!m_rdk) {
        emit moveCompleted(false, "Invalid RoboDK connection");
        return;
    }

    emit moveStarted();
    
    // 减少碰撞检测频率以提高性能
    m_rdk->setCollisionActive(false);  // 在真实机器人上禁用碰撞检测
    Mat pose;
    pose.FromXYZRPW(xyzwpr.data());

    qDebug() << "ExeMove:" << xyzwpr;

    //robot->setSpeed(5.0, 5.0, 10.0, 10.0);

    bool can_move = robot->MoveJ(pose);
    if (!can_move) {
        emit moveCompleted(false, "Target unreachable");
    } else {
        emit moveCompleted(true, "Movement completed");
    }
}

void RobotWorker::executeMoveJoints(Item robot, QVector<double> joints)
{
    qDebug() << "executeMoveJoints in thread:" << QThread::currentThreadId();

    if (m_stopFlag.loadRelaxed() == 1) {
        emit moveCompleted(false, "Movement stopped");
        return;
    }

    m_stopFlag.storeRelaxed(0);

    if (!robot) {
        emit moveCompleted(false, "Invalid robot object");
        return;
    }

    if (!m_rdk) {
        emit moveCompleted(false, "Invalid RoboDK connection");
        return;
    }

    emit moveStarted();

    tJoints robot_joints;
    for (int i = 0; i < qMin(joints.size(), 6); i++) {
        robot_joints.Data()[i] = joints[i];
    }

    // 减少碰撞检测频率以提高性能
    m_rdk->setCollisionActive(false);  // 在真实机器人上禁用碰撞检测

    //robot->setSpeed(5.0, 5.0, 10.0, 10.0);

    bool can_move = robot->MoveJ(robot_joints);

    if (!can_move) {
        emit moveCompleted(false, "Target unreachable");
    } else {
        emit moveCompleted(true, "Movement completed");
    }
}
