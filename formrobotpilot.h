#ifndef FORMROBOTPILOT_H
#define FORMROBOTPILOT_H

#include <QWidget>
#include <QDateTime>
#include <QTcpServer>
#include <QTcpSocket>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QVariant>
#include <QVector>
#include <QNetworkInterface>
#include <qtimer.h>
#include <qdebug.h>
#include "irobodk.h"
#include "iitem.h"
#include <qmessagebox.h>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QThread>
#include <QAtomicInt>
#include <QSettings>

#include "pose_sort.h"
#include "robotworker.h"

using trajsort::Pose;
using trajsort::PoseWithIndex;
using trajsort::sortPoses;
using trajsort::sortPosesWithIndex;
using trajsort::totalPosePathLength;

namespace Ui {
class FormRobotPilot;
}

// 单个雷达设备结构
struct RadarDevice {
    QSerialPort* serial = nullptr;
    QByteArray recvBuffer;
    quint8 slaveAddress = 0;      // Modbus 从机地址
    QString portName;
    QTimer* timer = nullptr;
    bool addressSet = false;      // 是否已成功设置地址
    QString RobotName;         // 关联的机器人（Robot1 或 Robot2）
};

class FormRobotPilot : public QWidget
{
    Q_OBJECT

public:

    explicit FormRobotPilot(RoboDK *rdk, QWidget *parent = nullptr);
    ~FormRobotPilot();

    bool SelectRobot();

    void IncrementalMove(int id, double sense);

private:

    void setup_btn_joints();

    void setup_btn_cartesian();

    bool moveToCartesian(const QVector<double>& values);

    bool Robot1moveToCartesian(const QVector<double>& values);

    bool Robot2moveToCartesian(const QVector<double>& values);

    bool moveToJoints(const QVector<double>& values);

    bool PlanningPosition(QVector<double>& values);

    bool SelectRobotByName(const QString& name);  // 新增：通过名称选择机器人

    bool SelectAllRobotByNames(const QString& name1, const QString& name2);  // 新增：通过名称选择机器人

    bool SetSpeed(double speed_linear, double speed_joints, double accel_linear, double accel_joints);

    bool ReadTargetInputs(QVector<double>& target);
    void SendRadarCommand(const QString& command);

    // 新增功能方法
    void ClearAllTargets();  // 清空所有目标点
    QString GetToolPosition();  // 获取工具坐标位置
    QString GetJointPositions();  // 获取关节位置

    QString GetToolPosition1();  // 获取工具坐标位置
    QString GetJointPositions1();  // 获取关节位置

    QString GetToolPosition2();  // 获取工具坐标位置
    QString GetJointPositions2();  // 获取关节位置
    
    // 多机械臂支持
    bool SetActiveRobot(const QString& robotName);  // 设置当前活跃机械臂
    Item GetRobotByName(const QString& name);  // 通过名称获取机械臂
    QStringList GetAvailableRobots();  // 获取所有可用机械臂
    double JointsDistance(const tJoints& j1, const tJoints& j2);
    double PoseDistance(const Mat& p1, const Mat& p2);
    bool ExecuteRobot(QVector<double> &target);

    void GetDistance();
    void SetDistance();

signals:
    void requestMoveCartesianWorker1(Item robot, QVector<double> xyzwpr, int mode);
    void requestMoveCartesianWorker2(Item robot, QVector<double> xyzwpr, int mode);
    void requestMoveJointsWorker1(Item robot, QVector<double> joints);
    void requestMoveJointsWorker2(Item robot, QVector<double> joints);
    void requestSetSpeedWorker1(Item robot, double speed_linear, double speed_joints, double accel_linear, double accel_joints);
    void requestSetSpeedWorker2(Item robot, double speed_linear, double speed_joints, double accel_linear, double accel_joints);
    void requestStop(bool stop);

private slots:

    void on_btnSelectRobot_clicked();

    void on_radCartesianTool_clicked();
    void on_radJoints_clicked();
    void on_radCartesianReference_clicked();

    void on_btnTXn_clicked();
    void on_btnTYn_clicked();
    void on_btnTZn_clicked();
    void on_btnRXn_clicked();
    void on_btnRYn_clicked();
    void on_btnRZn_clicked();
    void on_btnTXp_clicked();
    void on_btnTYp_clicked();
    void on_btnTZp_clicked();
    void on_btnRXp_clicked();
    void on_btnRYp_clicked();
    void on_btnRZp_clicked();

    void on_chkRunOnRobot_clicked(bool checked);
    
    void updateCartesianPosition();

    void on_btnPlanning_clicked();

    void on_btnExecute_clicked();

    void on_btnGetPos_clicked();

    void onClientDisconnected();

    void onNewConnection();

    void onReadyRead();

    void Slot_BtnStop();

    void Slot_LineEdit(const QString &Text);

    void RonConnected();

    void RonDisconnected();

    void RonReadyRead();

    void RonError(QAbstractSocket::SocketError error);

    void Slot_RadarTimeOut();

    void onMoveCompleted(bool success, QString message);
    void onMoveStarted();

private:
    Ui::FormRobotPilot *ui;

    RoboDK *RDK;

    Item Robot, Product, Robot1, Robot2;

    QTimer* m_timer;

    QTimer* m_RadarTimer;

    QTcpServer* m_tcpServer;

    QTcpSocket* m_clientSocket;

    QTcpSocket* m_Radarsocket;

    QMap<Item, std::vector<PoseWithIndex>> m_MapTargetPoints;
    QMap<Item, QVector<Item>> m_MapRobotTarget;

    int m_pointNumL = 0;
    int m_pointNumR = 0;
    QAtomicInt m_Stop;
    QMap<QString, QString> m_MapRobotAndBase;

    QString m_CurrentPath;

    int m_Distance;
    int m_StopType;
    int m_nRadarNum;
    int m_RadarPort;

    QString m_RadarHost;
    QString m_RadarResumeCommand;
    QString m_RadarStopCommand;

    bool m_isAllRobot, m_bIsRealRobot;

    // 工作线程相关
    QThread* m_workerThread1;
    QThread* m_workerThread2;
    RobotWorker* m_robotWorker1;
    RobotWorker* m_robotWorker2;
    QAtomicInt m_robot1Moving;
    QAtomicInt m_robot2Moving;
    QByteArray m_tcpRecvBuffer;

    // QVector<QString> m_VecComNum;
    QString m_Robot1ComNum, m_Robot2ComNum;
};

#endif // FORMROBOTPILOT_H
