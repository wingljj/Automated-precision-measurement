#include "formrobotpilot.h"
#include "ui_formrobotpilot.h"
#include <QWidget>
#include <QVector>
#include <QTimer>
#include <QTcpServer>
#include <QTcpSocket>
#include <QTextCursor>

/**
 * @brief 机器人控制面板构造函数
 * @param rdk RoboDK主程序接口指针
 * @param parent 父窗口指针
 */
FormRobotPilot::FormRobotPilot(RoboDK *rdk, QWidget *parent) : QWidget(parent),
    ui(new Ui::FormRobotPilot),
    m_tcpServer(new QTcpServer(this)),
    m_clientSocket(nullptr)
	, m_Radarsocket(nullptr)
    , m_Stop(0)
    , m_robot1Moving(0)
    , m_robot2Moving(0)
    , m_workerThread1(nullptr)
    , m_workerThread2(nullptr)
    , m_robotWorker1(nullptr)
    , m_robotWorker2(nullptr)
    , m_isAllRobot(false)
{
    // 保存RoboDK接口指针
    RDK = rdk;  
    // 初始化机器人对象为空
    Robot = nullptr;  

    Robot1 = nullptr;
    Robot2 = nullptr;

    Product = nullptr;

    Product = RDK->getItem("product", IItem::ITEM_TYPE_OBJECT);
    // 初始化UI界面
    ui->setupUi(this);  
    ui->label_ZBX->hide();
    ui->label_Robot1->hide();
    ui->widget_2->hide();

    // 设置关闭时自动删除
    setAttribute(Qt::WA_DeleteOnClose);  
    // 设置为独立窗口
    setWindowFlags(windowFlags() | Qt::Window);  
    // 默认选择工具坐标系模式
    ui->radCartesianTool->click();  

    ui->spnStep->setValue(5);

    ui->spnStep->setDecimals(6);
    // 尝试选择机器人
    //SelectRobot();  
    // 初始化定时器(100ms间隔，每秒10次)
    m_timer = new QTimer(this);
    connect(m_timer, &QTimer::timeout, this, &FormRobotPilot::updateCartesianPosition);
    m_timer->start(500);

    m_RadarTimer = new QTimer(this);
    connect(m_RadarTimer, &QTimer::timeout, this, &FormRobotPilot::Slot_RadarTimeOut);

    // 连接规划按钮信号槽
    connect(ui->btn_Planning, &QPushButton::clicked, this, &FormRobotPilot::on_btnPlanning_clicked);
    // 连接执行按钮信号槽
    connect(ui->btn_Execute, &QPushButton::clicked, this, &FormRobotPilot::on_btnExecute_clicked);
    // 初始化TCP服务器
    connect(m_tcpServer, &QTcpServer::newConnection, this, &FormRobotPilot::onNewConnection);

    connect(ui->btn_GetPos, &QPushButton::clicked, this, &FormRobotPilot::on_btnGetPos_clicked);

    connect(ui->btn_Stop, &QPushButton::clicked, this, &FormRobotPilot::Slot_BtnStop);

    connect(ui->lineEdit_RadarDistance, &QLineEdit::textChanged, this, &FormRobotPilot::Slot_LineEdit);

    if (!m_tcpServer->listen(QHostAddress::LocalHost, 8866)) 
    {
        ui->labelServerStatus->setText("Server failed: " + m_tcpServer->errorString());
    } 
    else 
    {
        ui->labelServerStatus->setText("Server running on: " + 
                                     m_tcpServer->serverAddress().toString() + ":" + 
                                     QString::number(m_tcpServer->serverPort()));
    }
    ui->labelClientStatus->setText("No client connected");

    m_CurrentPath = QCoreApplication::applicationDirPath();

    m_Radarsocket = new QTcpSocket(this);

	m_Radarsocket->connectToHost("169.254.0.66", 7);

    m_Radarsocket->write("Light:3;");

    connect(m_Radarsocket, &QTcpSocket::connected, this, &FormRobotPilot::RonConnected);
    connect(m_Radarsocket, &QTcpSocket::disconnected, this, &FormRobotPilot::RonDisconnected);
    connect(m_Radarsocket, &QTcpSocket::readyRead, this, &FormRobotPilot::RonReadyRead);
    connect(m_Radarsocket, QOverload<QAbstractSocket::SocketError>::of(&QAbstractSocket::errorOccurred),
        this, &FormRobotPilot::RonError);

    GetDistance();

    // 初始化雷达设备
    discoverAndSetupDevices();

    // 创建工作线程1
    m_workerThread1 = new QThread(this);
    m_robotWorker1 = new RobotWorker(RDK);
    m_robotWorker1->moveToThread(m_workerThread1);

    connect(this, &FormRobotPilot::requestMoveCartesian,
            m_robotWorker1, &RobotWorker::executeMoveCartesian);
    connect(this, &FormRobotPilot::requestMoveJoints,
            m_robotWorker1, &RobotWorker::executeMoveJoints);
    connect(this, &FormRobotPilot::requestStop,
            m_robotWorker1, &RobotWorker::stopMovement);
    connect(m_robotWorker1, &RobotWorker::moveCompleted,
            this, &FormRobotPilot::onMoveCompleted);
    connect(m_robotWorker1, &RobotWorker::moveStarted,
            this, &FormRobotPilot::onMoveStarted);
    connect(m_workerThread1, &QThread::finished,
            m_robotWorker1, &QObject::deleteLater);

    m_workerThread1->start();

    // 创建工作线程2
    m_workerThread2 = new QThread(this);
    m_robotWorker2 = new RobotWorker(RDK);
    m_robotWorker2->moveToThread(m_workerThread2);

    connect(this, &FormRobotPilot::requestMoveCartesian,
            m_robotWorker2, &RobotWorker::executeMoveCartesian);
    connect(this, &FormRobotPilot::requestMoveJoints,
            m_robotWorker2, &RobotWorker::executeMoveJoints);
    connect(this, &FormRobotPilot::requestStop,
            m_robotWorker2, &RobotWorker::stopMovement);
    connect(m_robotWorker2, &RobotWorker::moveCompleted,
            this, &FormRobotPilot::onMoveCompleted);
    connect(m_robotWorker2, &RobotWorker::moveStarted,
            this, &FormRobotPilot::onMoveStarted);
    connect(m_workerThread2, &QThread::finished,
            m_robotWorker2, &QObject::deleteLater);

    m_workerThread2->start();

    qDebug() << "Worker threads started";

    connect(ui->textEditLog, &QTextEdit::textChanged, [this]() {
        QTextCursor cursor = ui->textEditLog->textCursor();
        cursor.movePosition(QTextCursor::End);
        ui->textEditLog->setTextCursor(cursor);
        });
}
/**
 * @brief 机器人控制面板析构函数
 */
FormRobotPilot::~FormRobotPilot()
{
    delete ui;  // 释放UI资源
}
/**
 * @brief 选择机器人按钮点击事件处理函数
 */
void FormRobotPilot::on_btnSelectRobot_clicked()
{
    if (SelectRobot())
    {
        RDK->ShowMessage(QString::fromLocal8Bit("选择成功"));
    }
    else
    {
        RDK->ShowMessage(QString::fromLocal8Bit("选择失败"));
    }
}
/**
 * @brief 选择机器人函数
 * @return 是否成功选择机器人
 */
bool FormRobotPilot::SelectRobot()
{
    // 获取所有机器人列表
    QList<Item> all_robots = RDK->getItemList(IItem::ITEM_TYPE_ROBOT);
    // 检查是否有可用机器人
    if (all_robots.length() == 0)
    {
        ui->lblRobot->setText(QString::fromLocal8Bit("加载机器人"));
        return false;
    } 
    else
    {
        // 弹出选择对话框让用户选择机器人
        Robot = RDK->ItemUserPick(QString::fromLocal8Bit("选择机器人"), IItem::ITEM_TYPE_ROBOT);
    }
    // 检查是否成功选择机器人
    bool robot_is_selected = (Robot != nullptr);
    if (robot_is_selected)
    {
        ui->lblRobot->setText(QString::fromLocal8Bit("已选择机器人: ") + Robot->Name());  // 显示选择的机器人名称
    }
    else
    {
        ui->lblRobot->setText(QString::fromLocal8Bit("未选择机器人"));  // 显示未选择状态
    }
    ui->widget_2->hide();
    ui->label_Robot1->hide();
    ui->label_ZBX->hide();
    m_isAllRobot = false;
    return robot_is_selected;
}
/**
 * @brief 设置按钮文本为关节运动模式(J1-J6)
 */
void FormRobotPilot::setup_btn_joints()
{
    ui->btnTXn->setText("J1-");
    ui->btnTXp->setText("J1+");
    ui->btnTYn->setText("J2-");
    ui->btnTYp->setText("J2+");
    ui->btnTZn->setText("J3-");
    ui->btnTZp->setText("J3+");
    ui->btnRXn->setText("J4-");
    ui->btnRXp->setText("J4+");
    ui->btnRYn->setText("J5-");
    ui->btnRYp->setText("J5+");
    ui->btnRZn->setText("J6-");
    ui->btnRZp->setText("J6+");

    ui->label_01->setText("J1");
    ui->label_02->setText("J2");
    ui->label_03->setText("J3");
    ui->label_04->setText("J4");
    ui->label_05->setText("J5");
    ui->label_06->setText("J6");

    ui->label_5->setText("J1");
    ui->label_6->setText("J2");
    ui->label_7->setText("J3");
    ui->label_8->setText("J4");
    ui->label_9->setText("J5");
    ui->label_10->setText("J6");
}
/**
 * @brief 设置按钮文本为笛卡尔运动模式(Tx-Tz, Rx-Rz)
 */
void FormRobotPilot::setup_btn_cartesian()
{
    ui->btnTXn->setText("Tx-");
    ui->btnTXp->setText("Tx+");
    ui->btnTYn->setText("Ty-");
    ui->btnTYp->setText("Ty+");
    ui->btnTZn->setText("Tz-");
    ui->btnTZp->setText("Tz+");
    ui->btnRXn->setText("Rx-");
    ui->btnRXp->setText("Rx+");
    ui->btnRYn->setText("Ry-");
    ui->btnRYp->setText("Ry+");
    ui->btnRZn->setText("Rz-");
    ui->btnRZp->setText("Rz+");

    ui->label_01->setText("X");
    ui->label_02->setText("Y");
    ui->label_03->setText("Z");
    ui->label_04->setText("Rx");
    ui->label_05->setText("Ry");
    ui->label_06->setText("Rz");

    ui->label_5->setText("X");
    ui->label_6->setText("Y");
    ui->label_7->setText("Z");
    ui->label_8->setText("Rx");
    ui->label_9->setText("Ry");
    ui->label_10->setText("Rz");
}
/**
 * @brief 参考坐标系单选按钮点击事件
 */
void FormRobotPilot::on_radCartesianReference_clicked()
{
    setup_btn_cartesian();
}
/**
 * @brief 工具坐标系单选按钮点击事件
 */
void FormRobotPilot::on_radCartesianTool_clicked()
{
    setup_btn_cartesian();
}
/**
 * @brief 关节运动模式单选按钮点击事件
 */
void FormRobotPilot::on_radJoints_clicked()
{
    setup_btn_joints();
}

void FormRobotPilot::on_btnTXn_clicked()
{ 
    IncrementalMove(0, -1);
}
void FormRobotPilot::on_btnTYn_clicked()
{ 
    IncrementalMove(1, -1);
}
void FormRobotPilot::on_btnTZn_clicked()
{ 
    IncrementalMove(2, -1);
}
void FormRobotPilot::on_btnRXn_clicked()
{ 
    IncrementalMove(3, -1); 
}
void FormRobotPilot::on_btnRYn_clicked()
{
    IncrementalMove(4, -1);
}
void FormRobotPilot::on_btnRZn_clicked()
{ 
    IncrementalMove(5, -1);
}

void FormRobotPilot::on_btnTXp_clicked()
{ 
    IncrementalMove(0, +1); 
}
void FormRobotPilot::on_btnTYp_clicked()
{ 
    IncrementalMove(1, +1);
}
void FormRobotPilot::on_btnTZp_clicked()
{ 
    IncrementalMove(2, +1);
}
void FormRobotPilot::on_btnRXp_clicked()
{ 
    IncrementalMove(3, +1);
}
void FormRobotPilot::on_btnRYp_clicked()
{ 
    IncrementalMove(4, +1);
}
void FormRobotPilot::on_btnRZp_clicked()
{ 
    IncrementalMove(5, +1);
}
/**
 * @brief 执行增量运动
 * @param id 运动轴索引(0-5对应Tx-Tz,Rx-Rz或J1-J6)
 * @param sense 运动方向(+1或-1)
 */
void FormRobotPilot::IncrementalMove(int id, double sense)
{
    if (Robot == nullptr)
    {
        return;
    }
    double step = sense * ui->spnStep->value();
    bool is_joint_move = ui->radJoints->isChecked();
    if (is_joint_move)
    {
        // 选择Joints
        tJoints joints = Robot->Joints();
        if (id >= joints.Length())
        {
            return;
        }
        joints.Data()[id] = joints.Data()[id] + step;
        bool can_move = Robot->MoveL(joints);
        if (!can_move)
        {
            RDK->ShowMessage(QString::fromLocal8Bit("当前位置不可达"), false);
        }
    } 
    else
    {
        if (id < 0 || id >= 6)
        {
            return;
        }
        tXYZWPR xyzwpr;
        for (int i=0; i<6; i++)
        {
            xyzwpr[i] = 0;
        }
        xyzwpr[id] = step;
        Mat pose_increment;
        pose_increment.FromXYZRPW(xyzwpr);
        Mat pose_robot = Robot->Pose();
        Mat pose_robot_new;
        bool is_tcp_relative_move = ui->radCartesianTool->isChecked();
        if (is_tcp_relative_move)
        {
            // 选择Tool

            pose_robot_new = pose_robot * pose_increment;
        }
        else 
        {
            // 选择Reference

            Mat transformation_axes(pose_robot);
            transformation_axes.setPos(0, 0, 0);
            Mat movement_pose_aligned = transformation_axes.inv() * pose_increment * transformation_axes;
            pose_robot_new = pose_robot * movement_pose_aligned;
        }
        bool canmove = Robot->MoveL(pose_robot_new);
        if (!canmove)
        {
            RDK->ShowMessage(QString::fromLocal8Bit("当前位置不可达"), false);
        }
    }
    RDK->Render();
}

/**
 * @brief 更新坐标显示
 */
void FormRobotPilot::updateCartesianPosition()
{
    //if (!Robot)
    //{
    //    return;
    //}

    RDK->Render();

    if (ui->radJoints->isChecked())
    {
        if (!m_isAllRobot && Robot)
        {
            // 关节模式：显示关节角度
            tJoints joints = Robot->Joints();
            ui->lineEdit_X->setText(QString::number(joints.Data()[0], 'f', 6));
            ui->lineEdit_Y->setText(QString::number(joints.Data()[1], 'f', 6));
            ui->lineEdit_Z->setText(QString::number(joints.Data()[2], 'f', 6));
            ui->lineEdit_Rx->setText(QString::number(joints.Data()[3], 'f', 6));
            ui->lineEdit_Ry->setText(QString::number(joints.Data()[4], 'f', 6));
            ui->lineEdit_Rz->setText(QString::number(joints.Data()[5], 'f', 6));
        }
        else if (Robot1 && Robot2)
        {
            tJoints joints1 = Robot1->Joints();
            ui->lineEdit_X->setText(QString::number(joints1.Data()[0], 'f', 6));
            ui->lineEdit_Y->setText(QString::number(joints1.Data()[1], 'f', 6));
            ui->lineEdit_Z->setText(QString::number(joints1.Data()[2], 'f', 6));
            ui->lineEdit_Rx->setText(QString::number(joints1.Data()[3], 'f', 6));
            ui->lineEdit_Ry->setText(QString::number(joints1.Data()[4], 'f', 6));
            ui->lineEdit_Rz->setText(QString::number(joints1.Data()[5], 'f', 6));

            tJoints joints2 = Robot2->Joints();
            ui->lineEdit_X_3->setText(QString::number(joints2.Data()[0], 'f', 6));
            ui->lineEdit_Y_3->setText(QString::number(joints2.Data()[1], 'f', 6));
            ui->lineEdit_Z_3->setText(QString::number(joints2.Data()[2], 'f', 6));
            ui->lineEdit_Rx_3->setText(QString::number(joints2.Data()[3], 'f', 6));
            ui->lineEdit_Ry_3->setText(QString::number(joints2.Data()[4], 'f', 6));
            ui->lineEdit_Rz_3->setText(QString::number(joints2.Data()[5], 'f', 6));
        }
    } 
    else
    {
        if (!m_isAllRobot && Robot)
        {
            // 笛卡尔模式：显示TCP位姿
            Mat pose = Robot->Pose();
            tXYZWPR xyzwpr;
            pose.ToXYZRPW(xyzwpr);
            ui->lineEdit_X->setText(QString::number(xyzwpr[0], 'f', 6));
            ui->lineEdit_Y->setText(QString::number(xyzwpr[1], 'f', 6));
            ui->lineEdit_Z->setText(QString::number(xyzwpr[2], 'f', 6));
            ui->lineEdit_Rx->setText(QString::number(xyzwpr[3], 'f', 6));
            ui->lineEdit_Ry->setText(QString::number(xyzwpr[4], 'f', 6));
            ui->lineEdit_Rz->setText(QString::number(xyzwpr[5], 'f', 6));
        } 
        else if (Robot1 && Robot2)
        {
            // 笛卡尔模式：显示TCP位姿
            Mat pose1 = Robot1->Pose();
            tXYZWPR xyzwpr1;
            pose1.ToXYZRPW(xyzwpr1);
            ui->lineEdit_X->setText(QString::number(xyzwpr1[0], 'f', 6));
            ui->lineEdit_Y->setText(QString::number(xyzwpr1[1], 'f', 6));
            ui->lineEdit_Z->setText(QString::number(xyzwpr1[2], 'f', 6));
            ui->lineEdit_Rx->setText(QString::number(xyzwpr1[3], 'f', 6));
            ui->lineEdit_Ry->setText(QString::number(xyzwpr1[4], 'f', 6));
            ui->lineEdit_Rz->setText(QString::number(xyzwpr1[5], 'f', 6));

            // 笛卡尔模式：显示TCP位姿
            Mat pose2 = Robot2->Pose();
            tXYZWPR xyzwpr2;
            pose2.ToXYZRPW(xyzwpr2);
            ui->lineEdit_X_3->setText(QString::number(xyzwpr2[0], 'f', 6));
            ui->lineEdit_Y_3->setText(QString::number(xyzwpr2[1], 'f', 6));
            ui->lineEdit_Z_3->setText(QString::number(xyzwpr2[2], 'f', 6));
            ui->lineEdit_Rx_3->setText(QString::number(xyzwpr2[3], 'f', 6));
            ui->lineEdit_Ry_3->setText(QString::number(xyzwpr2[4], 'f', 6));
            ui->lineEdit_Rz_3->setText(QString::number(xyzwpr2[5], 'f', 6));
        }
    }
}

/**
 * @brief 实际机器人运行模式复选框点击事件
 * @param checked 是否选中
 */
/**
 * @brief 处理新TCP连接
 */
void FormRobotPilot::onNewConnection()
{
    if (m_clientSocket) 
    {
        m_clientSocket->disconnectFromHost();
    }
    
    m_clientSocket = m_tcpServer->nextPendingConnection();
    connect(m_clientSocket, &QTcpSocket::readyRead, this, &FormRobotPilot::onReadyRead);
    connect(m_clientSocket, &QTcpSocket::disconnected, this, &FormRobotPilot::onClientDisconnected);
    
    ui->labelClientStatus->setText(QString::fromLocal8Bit("已连接: ") +
                                 m_clientSocket->peerAddress().toString() + ":" + 
                                 QString::number(m_clientSocket->peerPort()));
}

/**
 * @brief 处理客户端数据接收
 */
void FormRobotPilot::onReadyRead()
{
    QByteArray data = m_clientSocket->readAll();
    QString Allmessage = QString::fromLocal8Bit(data).trimmed();
    QStringList MessageList = Allmessage.split("\r\n");
    for (QString message : MessageList)
    {
        if (!message.isEmpty())
        {
            ui->textEditLog->append("[" + QDateTime::currentDateTime().toString("hh:mm:ss") + "] Received: " + message);
            QStringList parts = message.split("_");
            if (parts.size() == 8) 
            {
                QString command = parts[0];
                // 通过指定模式和数据直接进行移动
                if (command == "E")
                {
                    QString mode = parts[1];
                    bool ok;
                    QVector<double> values;
                    // 解析6个数值参数
                    for (int i = 2; i < 8; ++i) 
                    {
                        values.append(parts[i].toDouble(&ok));
                        if (!ok)
                            break;
                    }
                    if (ok && values.size() == 6) 
                    {
                        if (mode == "T") 
                        {
                            // 工具坐标系运动
                            ui->radCartesianTool->click();
                            moveToCartesian(values);
                            m_clientSocket->write("Tool move executed");
                        } 
                        else if (mode == "T1")
                        {
                            // 工具坐标系运动
                            ui->radCartesianTool->click();
                            Robot1moveToCartesian(values);
                            m_clientSocket->write("Tool move executed");
                        }
                        else if (mode == "T2")
                        {
                            // 工具坐标系运动
                            ui->radCartesianTool->click();
                            Robot2moveToCartesian(values);
                            m_clientSocket->write("Tool move executed");
                        }
                        else if (mode == "R") 
                        {
                            // 参考坐标系运动
                            ui->radCartesianReference->click();
                            moveToCartesian(values);
                            m_clientSocket->write("Reference move executed");
                        }
                        else if (mode == "J") 
                        {
                            // 关节空间运动
                            ui->radJoints->click();
                            moveToJoints(values);
                            m_clientSocket->write("Joints move executed");
                        }
                        else 
                        {
                            // 无效模式
                            m_clientSocket->write("Invalid mode");
                        }
                    }
                    else
                    {
                        // 无效数据
                        m_clientSocket->write("Invalid data");
                    }
                }
                // 通过指定模式规划（支持多机械臂）
                else if(command == "P")
                {
                    QString mode = parts[1];
                    bool ok;
                    QVector<double> values;
                    // 解析6个数值参数
                    for (int i = 2; i < 8; i++)
                    {
                        values.append(parts[i].toDouble(&ok));
                        if (!ok)
                            break;
                    }
                    if (ok && values.size() == 6)
                    {
                        ui->radCartesianTool->click();
                        qDebug() << "Start Planning";
                        PlanningPosition(values);
                    }
                    else
                    {
                        m_clientSocket->write("Invalid data");
                    }
                }
            }
            else if (parts.size() == 3)
            {
                QString command = parts[0];
                QString Execute = parts[1];
                if (command == "M")
                {
                    // 加载模型
                    if (Execute == "Load")
                    {
                        // 加载模型命令格式: M Load filename.extension
                        QString filename = parts[2];
                        Item loadedItem = RDK->AddFile(filename);
                        if (RDK->Valid(loadedItem))
                        {
                            m_clientSocket->write(QString("Model loaded: %1").arg(loadedItem->Name()).toLocal8Bit());
                        }
                        else
                        {
                            m_clientSocket->write("Failed to load model");
                        }
                    }
                    // 隐藏模型
                    else if (Execute == "Hide")
                    {
                        // 隐藏模型命令格式: M Hide itemName
                        QString itemName = parts[2];
                        Item item = RDK->getItem(itemName);
                        if (RDK->Valid(item))
                        {
                            item->setVisible(false);
                            m_clientSocket->write(QString("Model hidden: %1").arg(itemName).toLocal8Bit());
                        }
                        else
                        {
                            m_clientSocket->write("Item not found");
                        }
                    }
                    // 显示模型
                    else if (Execute == "Show")
                    {
                        // 显示模型命令格式: M Show itemName
                        QString itemName = parts[2];
                        Item item = RDK->getItem(itemName);
                        if (RDK->Valid(item))
                        {
                            item->setVisible(true);
                            m_clientSocket->write(QString("Model shown: %1").arg(itemName).toLocal8Bit());
                        }
                        else
                        {
                            m_clientSocket->write("Item not found");
                        }
                    }
                    // 获取模型姿态
                    else if (Execute == "GetPose")
                    {
                        // 获取模型位姿命令格式: M GetPose itemName
                        QString itemName = parts[2];
                        Item item = RDK->getItem(itemName);
                        if (RDK->Valid(item))
                        {
                            Mat pose = item->Pose();
                            tXYZWPR xyzwpr;
                            pose.ToXYZRPW(xyzwpr);
                            QString poseStr = QString("%1_%2_%3_%4_%5_%6")
                                .arg(xyzwpr[0]).arg(xyzwpr[1]).arg(xyzwpr[2])
                                .arg(xyzwpr[3]).arg(xyzwpr[4]).arg(xyzwpr[5]);
                            m_clientSocket->write(poseStr.toLocal8Bit());
                        }
                        else
                        {
                            m_clientSocket->write("Item not found");
                        }
                    }
                    // 打开工程
                    else if (Execute == "OpenProject")
                    {
                        // 打开工程命令格式: M OpenProject filename.rdk
                        QString filename = parts[2];
                        // 使用AddFile方法打开工程文件，返回新加载的工程项
                        Item newStation = RDK->AddFile(filename);
                        bool success = RDK->Valid(newStation);
                        if (success)
                        {
                            m_clientSocket->write(QString("Project opened: %1").arg(filename).toLocal8Bit());
                            // 重新选择机器人
                            //SelectRobot();
                        }
                        else
                        {
                            m_clientSocket->write("Failed to open project");
                        }
                    }
                    // 读取JSON文件
                    else if (Execute == "ReadJson")
                    {
                        // 读取JSON文件命令格式: M ReadJson filename.json
                        QString filename = parts[2];
                        bool success = ReadJsonFile(filename);
                        if (success)
                        {
                            m_clientSocket->write(QString("JSON file read successfully: %1").arg(filename).toLocal8Bit());
                        }
                        else
                        {
                            m_clientSocket->write(QString("Failed to read JSON file: %1").arg(filename).toLocal8Bit());
                        }
                    }
                }
                // 获取数据命令
                else if (command == "G")
                {
                    if (Execute == "T" &&  "Pos")
                    {
                        // 获取工具坐标位置命令格式: G T_Pos
                        QString toolPos = GetToolPosition();
                        m_clientSocket->write(toolPos.toLocal8Bit());
                    }
                    else if (Execute == "T1" && "Pos")
                    {
                        // 获取工具坐标位置命令格式: G T_Pos
                        QString toolPos = GetToolPosition1();
                        m_clientSocket->write(toolPos.toLocal8Bit());
                    }
                    else if (Execute == "T2" && "Pos")
                    {
                        // 获取工具坐标位置命令格式: G T_Pos
                        QString toolPos = GetToolPosition2();
                        m_clientSocket->write(toolPos.toLocal8Bit());
                    }
                    else if (Execute == "J" &&  "Pos")
                    {
                        // 获取关节位置命令格式: G J_Pos
                        QString jointPos = GetJointPositions();
                        m_clientSocket->write(jointPos.toLocal8Bit());
                    }
                    else if (Execute == "J1" && "Pos")
                    {
                        // 获取关节位置命令格式: G J_Pos
                        QString jointPos = GetJointPositions1();
                        m_clientSocket->write(jointPos.toLocal8Bit());
                    }
                    else if (Execute == "J2" && "Pos")
                    {
                        // 获取关节位置命令格式: G J_Pos
                        QString jointPos = GetJointPositions2();
                        m_clientSocket->write(jointPos.toLocal8Bit());
                    }
                    else if (Execute == "All" && "Pos")
                    {
                        QString toolPos = GetToolPosition();
                        QString jointPos = GetJointPositions();
                        QString AllPos = toolPos + ";" + jointPos;
                        m_clientSocket->write(AllPos.toLocal8Bit());
                    }
                    else
                    {
                        m_clientSocket->write("Invalid get command");
                    }
                }
                else if (command == "C")
                {
                    if (SelectAllRobotByNames(parts[1], parts[2]))
                    {
                        if (Robot1 != nullptr && Robot2 != nullptr)
                        {
                            ui->lblRobot->setText(QString::fromLocal8Bit("已选择机器人: ") + Robot1->Name() + " " + Robot2->Name());  // 显示选择的机器人名称
                        }
                        else
                        {
                            ui->lblRobot->setText(QString::fromLocal8Bit("未选择机器人"));  // 显示未选择状态
                        }
                    }
                }
                else
                {
                    m_clientSocket->write("Invalid model command");

                    QString aa = command + Execute + parts[2];
                    m_clientSocket->write(aa.toLocal8Bit());
                }
            }
            else if (parts.size() == 6)
            {
                QString command = parts[0];
                QString Execute = parts[1];
                if (Execute == "Rotate")
                {
                    // 旋转模型命令格式: M Rotate itemName rx ry rz
                    QString itemName = parts[2];
                    Item item = RDK->getItem(itemName);
                    if (RDK->Valid(item))
                    {
                        bool ok;
                        double rx = parts[3].toDouble(&ok);
                        double ry = parts[4].toDouble(&ok);
                        double rz = parts[5].toDouble(&ok);

                        if (ok)
                        {
                            Mat currentPose = item->Pose();
                            Mat rotation = Mat::rotx(rx) * Mat::roty(ry) * Mat::rotz(rz);
                            Mat newPose = currentPose * rotation;
                            item->setPose(newPose);
                            RDK->Render();
                            m_clientSocket->write(QString("Model rotated: %1").arg(itemName).toLocal8Bit());
                        }
                        else
                        {
                            m_clientSocket->write("Invalid rotation parameters");
                        }
                    }
                    else
                    {
                        m_clientSocket->write("Item not found");
                    }
                }
                else if (Execute == "Move")
                {
                    // 移动模型命令格式: M Move itemName x y z
                    QString itemName = parts[2];
                    Item item = RDK->getItem(itemName);
                    if (RDK->Valid(item))
                    {
                        bool ok;
                        double x = parts[3].toDouble(&ok);
                        double y = parts[4].toDouble(&ok);
                        double z = parts[5].toDouble(&ok);

                        if (ok)
                        {
                            Mat currentPose = item->Pose();
                            Mat translation = Mat::transl(x, y, z);
                            Mat newPose = currentPose * translation;
                            item->setPose(newPose);
                            RDK->Render();
                            m_clientSocket->write(QString("Model moved: %1").arg(itemName).toLocal8Bit());
                        }
                        else
                        {
                            m_clientSocket->write("Invalid move parameters");
                        }
                    }
                    else
                    {
                        m_clientSocket->write("Item not found");
                    }
                }
                else if (Execute == "Speed")
                {
                    SetSpeed(parts[2].toDouble(), parts[3].toDouble(), parts[4].toDouble(), parts[5].toDouble());
                }
            }
            else if (parts.size() == 9)
            {
                QString command = parts[0];
                QString Execute = parts[1];
                if (Execute == "SetPose")
                {
                    // 设置模型位姿命令格式: M SetPose itemName x y z rx ry rz
                    QString itemName = parts[2];
                    Item item = RDK->getItem(itemName);
                    if (RDK->Valid(item))
                    {
                        bool ok;
                        double x = parts[3].toDouble(&ok);
                        double y = parts[4].toDouble(&ok);
                        double z = parts[5].toDouble(&ok);
                        double rx = parts[6].toDouble(&ok);
                        double ry = parts[7].toDouble(&ok);
                        double rz = parts[8].toDouble(&ok);

                        if (ok)
                        {
                            tXYZWPR pose;
                            pose[0] = x; pose[1] = y; pose[2] = z;
                            pose[3] = rx; pose[4] = ry; pose[5] = rz;

                            Mat newPose;
                            newPose.FromXYZRPW(pose);
                            item->setPose(newPose);
                            RDK->Render();
                            m_clientSocket->write(QString("Model pose set: %1").arg(itemName).toLocal8Bit());
                        }
                        else
                        {
                            m_clientSocket->write("Invalid pose parameters");
                        }
                    }
                    else
                    {
                        m_clientSocket->write("Item not found");
                    }
                }
            }
            else if (parts.size() == 2)
            {
                QString command = parts[0];
                QString Execute = parts[1];
                if (command == "C")
                {
                    if (SelectRobotByName(Execute))
                    {
                        // 检查是否成功选择机器人
                        bool robot_is_selected = (Robot != nullptr);
                        if (robot_is_selected)
                        {
                            ui->lblRobot->setText(QString::fromLocal8Bit("已选择机器人:\n") + Robot->Name());  // 显示选择的机器人名称
                        }
                        else
                        {
                            ui->lblRobot->setText(QString::fromLocal8Bit("未选择机器人"));  // 显示未选择状态
                        }
                    }
                    else
                    {
                        QMessageBox::information(nullptr,
                            QString::fromLocal8Bit("提示"),
                            QString::fromLocal8Bit("名称错误"),
                            QMessageBox::Ok);
                    }
                }
                else if(command == "E")
                {
                    if (Execute == "Execute")
                    {
                        on_btnExecute_clicked();
                    }
                    if (Execute == "AllExecute")
                    {
                        qDebug() << QString::fromLocal8Bit("全部运行");
                        QVector<double> target;
                        ExecuteRobot(target);
                    }
                }
                else if (command == "G")
                {
                    QStringList AllName;
                    if (Execute == "GetNames")
				    {
                        AllName = RDK->getItemListNames(IItem::ITEM_TYPE_ROBOT);
                        m_clientSocket->write(AllName.join("_").toLocal8Bit());
				    }
                    else if (Execute == "BaseNames")
                    {
                        AllName = RDK->getItemListNames(IItem::ITEM_TYPE_FRAME);
					    m_clientSocket->write(AllName.join("_").toLocal8Bit());
				    }
				    else 
				    {
                        QMessageBox::information(nullptr,
                            QString::fromLocal8Bit("提示"),
                            QString::fromLocal8Bit("命令错误"),
                            QMessageBox::Ok);
				    }
			    }
                // 模型控制命令
                else if (command == "M")
                {
                    if (Execute == "List")
                    {
                        // 列出所有模型命令格式: M List
                        QStringList allItems = RDK->getItemListNames();
                        m_clientSocket->write(allItems.join("_").toLocal8Bit());
                    }
                    else if (Execute == "Clear")
                    {
                        // 清空所有目标点命令格式: M T_Clear
                        ClearAllTargets();
                        m_clientSocket->write("All targets cleared successfully");
                    }
                }
                else if (command == "U")
                {
                    if (Execute == "UseRobot")
                    {
                        ui->chkRunOnRobot->setCheckState(Qt::Checked);
                        on_chkRunOnRobot_clicked(true);
                    }
                    else
                    {
                        ui->chkRunOnRobot->setCheckState(Qt::Unchecked);
                        on_chkRunOnRobot_clicked(false);
                    }
                }
            }
            else
            {
                m_clientSocket->write("Invalid command format");
            }
        }
    }
}

void FormRobotPilot::Slot_BtnStop()
{
    if (m_Stop)
    {
        // 解除急停
        m_Stop = false;
        ui->btn_Stop->setText(QString::fromLocal8Bit("急停"));
        if (m_clientSocket)
        {
            m_clientSocket->write("CancelStop");
        }
        if (m_Radarsocket)
        {
            ui->textEditLog->append(QString::fromLocal8Bit("点击恢复按钮"));
            m_Radarsocket->write("Light:3;");
        }
    }
    else
    {
        // 触发急停
        //Robot->Stop();
        m_Stop = true;
        ui->btn_Stop->setText(QString::fromLocal8Bit("恢复"));
        if (m_clientSocket)
        {
            m_clientSocket->write("StopRunning");
        }
        if (m_Radarsocket)
        {
            ui->textEditLog->append(QString::fromLocal8Bit("点击急停按钮"));
            m_Radarsocket->write("Light:5;");
        }
    }
}

void FormRobotPilot::Slot_LineEdit(const QString& Text)
{
    m_Distance = Text.toInt();
    SetDistance();
}

void FormRobotPilot::RonConnected()
{
    ui->textEditLog->append(QString::fromLocal8Bit("测录雷达急停控制器连接成功"));
}

void FormRobotPilot::RonDisconnected()
{
    ui->textEditLog->append(QString::fromLocal8Bit("测录雷达急停控制器断开连接"));
}

void FormRobotPilot::RonReadyRead()
{
    QByteArray data = m_Radarsocket->readAll();
    QString StrData = QString::fromLocal8Bit(data).trimmed();
    if(StrData.contains("5"))
    {
        ui->textEditLog->append(QString::fromLocal8Bit("一、二号机械臂急停"));
        m_Stop = true;
        ui->btn_Stop->setText(QString::fromLocal8Bit("恢复"));
    }
    else if(StrData.contains("1"))
    {
        ui->textEditLog->append(QString::fromLocal8Bit("二号机械臂急停"));
        m_Stop = true;
        ui->btn_Stop->setText(QString::fromLocal8Bit("恢复"));
    }
    else if(StrData.contains("2"))
    {
        ui->textEditLog->append(QString::fromLocal8Bit("一号机械臂急停"));
        m_Stop = true;
        ui->btn_Stop->setText(QString::fromLocal8Bit("恢复"));
    }
    else if(StrData.contains("3"))
    {
        ui->textEditLog->append(QString::fromLocal8Bit("一、二号机械臂恢复"));
        m_Stop = false;
        ui->btn_Stop->setText(QString::fromLocal8Bit("急停"));
    }
}

void FormRobotPilot::RonError(QAbstractSocket::SocketError error)
{
    Q_UNUSED(error);
    ui->textEditLog->append(QString::fromLocal8Bit("测录雷达急停控制器连接错误"));
}

void FormRobotPilot::Slot_RadarTimeOut()
{
    if(m_nRadarNum >= 5)
    {
        m_Radarsocket->write("Light:5;");
        ui->textEditLog->append(QString::fromLocal8Bit("机械臂距离过近触发急停"));
        m_nRadarNum = 0;
    }
    else
    {
        m_nRadarNum = 0;
    }
}

/**
 * @brief 移动到笛卡尔空间目标位置
 */
bool FormRobotPilot::moveToCartesian(const QVector<double>& values)
{
    if (!Robot) 
        return false;
    tXYZWPR xyzwpr;
    for (int i = 0; i < 6; ++i) 
    {
        xyzwpr[i] = values[i];
    }
    qDebug() << QString::fromLocal8Bit("检查碰撞");
    // 启用碰撞检测
    RDK->setCollisionActive(true);
    Mat pose;
    pose.FromXYZRPW(xyzwpr);
    bool can_move = Robot->MoveJ(pose);
    if (!can_move)
    {
        // 检查是否是碰撞导致
        if (RDK->Collision(Robot, Product))  // 检查机器人与所有物体的碰撞 
        {
            RDK->ShowMessage(tr("Movement stopped due to collision!"), false);
            if (m_clientSocket)
            {
                m_clientSocket->write("Movement stopped due to collision!");
            }
            
            return false;
        }
        else 
        {
            RDK->ShowMessage(tr("The robot can't move to this location"), false);
            if (m_clientSocket)
            {
                m_clientSocket->write("The robot can't move to this location");
            }
            
            return false;
        }
    }
    RDK->Render();
    return true;
}
bool FormRobotPilot::Robot1moveToCartesian(const QVector<double>& values)
{
    if (!Robot1)
        return false;
    tXYZWPR xyzwpr;
    for (int i = 0; i < 6; ++i)
    {
        xyzwpr[i] = values[i];
    }
    qDebug() << QString::fromLocal8Bit("检查碰撞");
    // 启用碰撞检测
    RDK->setCollisionActive(true);
    Mat pose;
    pose.FromXYZRPW(xyzwpr);
    bool can_move = Robot1->MoveJ(pose);
    if (!can_move)
    {
        // 检查是否是碰撞导致
        if (RDK->Collision(Robot1, Product))  // 检查机器人与所有物体的碰撞 
        {
            RDK->ShowMessage(tr("Movement stopped due to collision!"), false);
            if (m_clientSocket)
            {
                m_clientSocket->write("Movement stopped due to collision!");
            }

            return false;
        }
        else
        {
            RDK->ShowMessage(tr("The robot can't move to this location"), false);
            if (m_clientSocket)
            {
                m_clientSocket->write("The robot can't move to this location");
            }

            return false;
        }
    }
    RDK->Render();
    return true;
}
bool FormRobotPilot::Robot2moveToCartesian(const QVector<double>& values)
{
    if (!Robot2)
        return false;
    tXYZWPR xyzwpr;
    for (int i = 0; i < 6; ++i)
    {
        xyzwpr[i] = values[i];
    }
    qDebug() << QString::fromLocal8Bit("检查碰撞");
    // 启用碰撞检测
    RDK->setCollisionActive(true);
    Mat pose;
    pose.FromXYZRPW(xyzwpr);
    bool can_move = Robot2->MoveJ(pose);
    if (!can_move)
    {
        // 检查是否是碰撞导致
        if (RDK->Collision(Robot2, Product))  // 检查机器人与所有物体的碰撞 
        {
            RDK->ShowMessage(tr("Movement stopped due to collision!"), false);
            if (m_clientSocket)
            {
                m_clientSocket->write("Movement stopped due to collision!");
            }

            return false;
        }
        else
        {
            RDK->ShowMessage(tr("The robot can't move to this location"), false);
            if (m_clientSocket)
            {
                m_clientSocket->write("The robot can't move to this location");
            }

            return false;
        }
    }
    RDK->Render();
    return true;
}
/**
 * @brief 移动到关节空间目标位置
 */
bool FormRobotPilot::moveToJoints(const QVector<double>& values)
{
    if (!Robot) 
        return false;
    tJoints joints;
    for (int i = 0; i < 6; ++i) 
    {
        joints.Data()[i] = values[i];
    }
    // 启用碰撞检测
    RDK->setCollisionActive(true);
    bool can_move = Robot->MoveJ(joints);
    if (!can_move)
    {
        // 检查是否是碰撞导致
        if (RDK->Collision(Robot, nullptr))
        {
            RDK->ShowMessage(tr("Movement stopped due to collision!"), false);
            if (m_clientSocket)
            {
                m_clientSocket->write("Movement stopped due to collision!");
            }
            
            return false;
        }
        else
        {
            RDK->ShowMessage(tr("The robot can't move to this location"), false);
            if (m_clientSocket)
            {
                m_clientSocket->write("The robot can't move to this location");
            }
            
            return false;
        }
    }
    RDK->Render();
    return true;
}

bool FormRobotPilot::PlanningPosition(QVector<double>& target)
{
    // 步骤1: 获取/切换到指定 Robot（使用您的 GetRobotByName）
    Item baseFrame = RDK->getItem(m_MapRobotAndBase.value(Robot->Name()), IItem::ITEM_TYPE_FRAME);
    
    // 检查是否有活跃的机械臂
    if (!Robot)
    {
        RDK->ShowMessage(QString::fromLocal8Bit("请先选择机械臂"), true);
        return false;
    }
    int mode = ui->radCartesianReference->isChecked() ? 0 :
        (ui->radCartesianTool->isChecked() ? 1 : 2);
    qDebug() << QString::fromLocal8Bit("可达性判断");
    std::vector<PoseWithIndex> posesWithIndex = m_MapTargetPoints.value(Robot);
    // 笛卡尔模式：使用 Mat (位姿)
    tXYZWPR tempxyzwpr;
    tempxyzwpr[0] = target[0];
    tempxyzwpr[1] = target[1];
    tempxyzwpr[2] = target[2];
    tempxyzwpr[3] = target[3];
    tempxyzwpr[4] = target[4];
    tempxyzwpr[5] = target[5];
    Mat temppose;
    temppose.FromXYZRPW(tempxyzwpr);  // 从头文件：Mat::FromXYZRPW
    tJoints TempJoints = Robot->SolveIK(temppose, nullptr, &Robot->PoseTool(), &Robot->PoseFrame());  // 求 IK 解
    bool reachable = TempJoints.Valid() && TempJoints.Length() > 0;

    if (!reachable)
    {
        qDebug() << QString::fromLocal8Bit("不可达");
        int CantIndex = -100 - posesWithIndex.size();
        if (m_clientSocket)
        {
            m_clientSocket->write(QString::number(CantIndex).toLocal8Bit());
        }
        
		return false;
    }
    else
    {
        int dof = TempJoints.Length();  // e.g., 6
        const double* jointsPtr = TempJoints.Values();  // 只读指针
        // 读所有关节 (循环)
        for (int i = 0; i < dof; ++i) 
        {
            double joint = jointsPtr[i];  // J1=i=0, J2=i=1, ...
            qDebug() << QString("J%1: %2°").arg(i + 1).arg(joint, 0, 'f', 2);
        }
    }

    qDebug() << QString::fromLocal8Bit("Target可达，开始排序");

    Pose PointPost;
    PointPost.x = target[0];
    PointPost.y = target[1];
    PointPost.z = target[2];
    PointPost.rx = target[3];
    PointPost.ry = target[4];
    PointPost.rz = target[5];

    PoseWithIndex posewithindex;
    posewithindex.index = posesWithIndex.size();
    posewithindex.pose = PointPost;
    posesWithIndex.push_back(posewithindex);
    const auto sortedWithIndex = sortPosesWithIndex(posesWithIndex, 0, 1.0, 0.0);
    m_MapTargetPoints.insert(Robot, sortedWithIndex);
    qDebug() << QString::fromLocal8Bit("清理界面target");
    for (Item target : m_MapRobotTarget.value(Robot))
    {
        target->Delete();
    }
    bool success = true;
    QVector<Item> VecTarget;
    QString OrderIndex = "Index:";
    // 重新添加排序后的Target
    qDebug() << QString::fromLocal8Bit("重新添加排序后的Target");
    for (int i = 0; i < sortedWithIndex.size(); i++)
    {
        // 笛卡尔模式：使用 Mat (位姿)
        tXYZWPR xyzwpr;
        xyzwpr[0] = sortedWithIndex[i].pose.x;
        xyzwpr[1] = sortedWithIndex[i].pose.y;
		xyzwpr[2] = sortedWithIndex[i].pose.z;
		xyzwpr[3] = sortedWithIndex[i].pose.rx;
		xyzwpr[4] = sortedWithIndex[i].pose.ry;
		xyzwpr[5] = sortedWithIndex[i].pose.rz;
        Mat pose;
        pose.FromXYZRPW(xyzwpr);  // 从头文件：Mat::FromXYZRPW

        if (!pose.Valid())
        {
            qDebug() << QString::fromLocal8Bit("位姿数据无效");
            RDK->ShowMessage(QString::fromLocal8Bit("位姿数据无效"), true);
            return false;
        }
        else
        {
            // 创建目标点名称
            QString targetName = QString("Target%1_%2").arg(Robot->Name().split(" ").last()).arg(i);
            // 创建空的 Target 项（根据 IRoboDK::AddTarget 签名）
            Item targetItem = RDK->AddTarget(targetName, baseFrame, Robot);
            // 判断添加状态
            if (!RDK->Valid(targetItem))
            {
                RDK->ShowMessage(QString::fromLocal8Bit("添加目标点失败"), true);
                return false;
            }
            targetItem->setPose(pose);  // IItem::setPose (确认存在于 API)
            ui->textEditLog->append("[" + QDateTime::currentDateTime().toString("hh:mm:ss") + "] Execute: " + QString::fromLocal8Bit("添加目标点：") + targetName);
            VecTarget.append(targetItem);
            OrderIndex.append(QString("_%1").arg(static_cast<int>(sortedWithIndex[i].index)));
            qDebug() << QString::fromLocal8Bit("成功添加：%1").arg(targetName);
        }
    }
    if (m_clientSocket)
    {
        m_clientSocket->write(OrderIndex.toLocal8Bit());
    }
    m_MapRobotTarget.insert(Robot, VecTarget);
    RDK->Render();
    return success;
}

bool FormRobotPilot::SelectRobotByName(const QString& name)
{
    if (name.isEmpty()) 
    {
        return false;
    }
    // 使用RoboDK API获取指定机器人
    Robot = RDK->getItem(name, IItem::ITEM_TYPE_ROBOT);
    bool success = RDK->Valid(Robot);
    if (success)
    {
        QString RobotName = Robot->Name();
        ui->lblRobot->setText(QString::fromLocal8Bit("已选择机器人: ") + RobotName);
        RDK->ShowMessage(tr("Robot selected: ") + name, false);
        // 更新当前关节/位姿显示
        updateCartesianPosition();
        QStringList AllName = RDK->getItemListNames(IItem::ITEM_TYPE_FRAME);
        for (const QString& name : AllName)
        {
            if (RobotName.split(" ").last() == name.split(" ").last())
            {
                m_MapRobotAndBase.insert(RobotName, name);
                break;
            }
        }
    }
    else
    {
        ui->lblRobot->setText(QString::fromLocal8Bit("未选择机器人: ") + name);
        RDK->ShowMessage(tr("Robot not found: ") + name, false);
    }
    RDK->Render();
    return success;
}

bool FormRobotPilot::SelectAllRobotByNames(const QString& name1, const QString& name2)
{
    if (name1.isEmpty() || name2.isEmpty())
    {
        return false;
    }

    Robot1 = RDK->getItem(name1, IItem::ITEM_TYPE_ROBOT);
    bool success1 = RDK->Valid(Robot1);
    if (success1)
    {
        QString RobotName = Robot1->Name();
        ui->lblRobot->setText(QString::fromLocal8Bit("已选择机器人:\n") + RobotName);
        QStringList AllName = RDK->getItemListNames(IItem::ITEM_TYPE_FRAME);
        for (const QString& name : AllName)
        {
            if (RobotName.split(" ").last() == name.split(" ").last())
            {
                m_MapRobotAndBase.insert(RobotName, name);
                break;
            }
        }
    }
    else
    {
        return false;
    }

    Robot2 = RDK->getItem(name2, IItem::ITEM_TYPE_ROBOT);
    bool success2 = RDK->Valid(Robot2);
    if (success2)
    {
        QString RobotName = Robot2->Name();
        ui->lblRobot->setText(QString::fromLocal8Bit("已选择机器人:\n") + RobotName);
        QStringList AllName = RDK->getItemListNames(IItem::ITEM_TYPE_FRAME);
        for (const QString& name : AllName)
        {
            if (RobotName.split(" ").last() == name.split(" ").last())
            {
                m_MapRobotAndBase.insert(RobotName, name);
                break;
            }
        }
    }
    else
    {
        return false;
    }

    RDK->Render();
    ui->widget_2->show();
    ui->label_Robot1->show();
    ui->label_ZBX->show();
    m_isAllRobot = true;
    return true;
}

bool FormRobotPilot::SetSpeed(double speed_linear, double speed_joints, double accel_linear, double accel_joints)
{
    if (Robot)
    {
        Robot->setSpeed(speed_linear, speed_joints, accel_linear, accel_joints);
    }
    if (Robot1) 
    {
        Robot1->setSpeed(speed_linear, speed_joints, accel_linear, accel_joints);
    }
    if (Robot2)
    {
        Robot2->setSpeed(speed_linear, speed_joints, accel_linear, accel_joints);
    }
    ui->textEditLog->setText(QString::fromLocal8Bit("线速度：%1，角速度：%2，线加速度：%3，角加速度：%4").arg(speed_linear).arg(speed_joints).arg(accel_linear).arg(accel_joints));
    return true;
}

/**
 * @brief 处理客户端断开连接
 */
void FormRobotPilot::onClientDisconnected()
{
    ui->labelClientStatus->setText("No client connected");
    m_clientSocket->deleteLater();
    m_clientSocket = nullptr;
}

/**
 * @brief 规划按钮点击事件处理函数
 */
void FormRobotPilot::on_btnPlanning_clicked()
{
    if (ui->lineEdit_X_2->text().isEmpty() || ui->lineEdit_Y_2->text().isEmpty()
        || ui->lineEdit_Z_2->text().isEmpty() || ui->lineEdit_Rx_2->text().isEmpty()
        || ui->lineEdit_Ry_2->text().isEmpty() || ui->lineEdit_Rz_2->text().isEmpty())
    {
        return;
    }
    // 获取当前LineEdit中的坐标值
    QVector<double> target(6);
    target[0] = ui->lineEdit_X_2->text().toDouble();
    target[1] = ui->lineEdit_Y_2->text().toDouble();
    target[2] = ui->lineEdit_Z_2->text().toDouble();
    target[3] = ui->lineEdit_Rx_2->text().toDouble();
    target[4] = ui->lineEdit_Ry_2->text().toDouble();
    target[5] = ui->lineEdit_Rz_2->text().toDouble();

    if (PlanningPosition(target))
    {
        RDK->ShowMessage(QString::fromLocal8Bit("目标点添加成功"), false);
    }
    else
    {
        RDK->ShowMessage(QString::fromLocal8Bit("目标点添加失败"), false);
    }
}

/**
 * @brief 执行按钮点击事件处理函数
 */
void FormRobotPilot::on_btnExecute_clicked()
{
    QVector<double> target;
    if (ui->lineEdit_X_2->text().isEmpty() || ui->lineEdit_Y_2->text().isEmpty()
        || ui->lineEdit_Z_2->text().isEmpty() || ui->lineEdit_Rx_2->text().isEmpty()
        || ui->lineEdit_Ry_2->text().isEmpty() || ui->lineEdit_Rz_2->text().isEmpty())
    {
        return;
    }
    target.append(ui->lineEdit_X_2->text().toDouble());
    target.append(ui->lineEdit_Y_2->text().toDouble());
    target.append(ui->lineEdit_Z_2->text().toDouble());
    target.append(ui->lineEdit_Rx_2->text().toDouble());
    target.append(ui->lineEdit_Ry_2->text().toDouble());
    target.append(ui->lineEdit_Rz_2->text().toDouble());
    bool moveSuccess;

    qDebug() << QString::fromLocal8Bit("按钮执行");

    if (ui->radJoints->isChecked())
    {
        moveSuccess = moveToJoints(target);
    }
    else
    {
        moveSuccess = moveToCartesian(target);
    }
}

void FormRobotPilot::on_btnGetPos_clicked()
{
    ui->lineEdit_X_2->setText(ui->lineEdit_X->text());
    ui->lineEdit_Y_2->setText(ui->lineEdit_Y->text());
    ui->lineEdit_Z_2->setText(ui->lineEdit_Z->text());
    ui->lineEdit_Rx_2->setText(ui->lineEdit_Rx->text());
    ui->lineEdit_Ry_2->setText(ui->lineEdit_Ry->text());
    ui->lineEdit_Rz_2->setText(ui->lineEdit_Rz->text());
}

void FormRobotPilot::on_chkRunOnRobot_clicked(bool checked)
{
    if (m_isAllRobot)
    {
        if (Robot1 && Robot2)
        {
            Robot1->Connect();
            Robot2->Connect();
        }
        else
        {
            RDK->ShowMessage(tr("Please Choose Robot"), false);
            ui->chkRunOnRobot->setCheckState(Qt::Unchecked);
            return;
        }
    }
    else
    {
        if (Robot)
        {
            Robot->Connect();
        }
        else
        {
            RDK->ShowMessage(tr("Please Choose Robot"), false);
            ui->chkRunOnRobot->setCheckState(Qt::Unchecked);
            return;
        }
    }

    if (checked)
    {
        RDK->setRunMode(RoboDK::RUNMODE_RUN_ROBOT);
        RDK->Render();
        RDK->ShowMessage(tr("Run Mode set to run the real robot. Make sure you are properly connected to the robot (select Connect-Connect Robot)"), false);
    } 
    else 
    {
        RDK->setRunMode(RoboDK::RUNMODE_SIMULATE);
        RDK->ShowMessage(tr("Run Mode set to simulate"), false);
    }
}

/**
 * @brief 读取JSON文件并处理数据
 * @param filename JSON文件路径
 * @return 是否成功读取和处理JSON文件
 */
bool FormRobotPilot::ReadJsonFile(const QString& filename)
{
    QFile file(filename);
    if (!file.open(QIODevice::ReadOnly))
    {
        ui->textEditLog->append("[" + QDateTime::currentDateTime().toString("hh:mm:ss") + "] Error: " + 
                               QString::fromLocal8Bit("无法打开JSON文件: ") + filename);
        return false;
    }

    QByteArray jsonData = file.readAll();
    file.close();

    QJsonParseError parseError;
    QJsonDocument jsonDoc = QJsonDocument::fromJson(jsonData, &parseError);
    
    if (parseError.error != QJsonParseError::NoError)
    {
        ui->textEditLog->append("[" + QDateTime::currentDateTime().toString("hh:mm:ss") + "] Error: " + 
                               QString::fromLocal8Bit("JSON解析错误: ") + parseError.errorString());
        return false;
    }

    if (!jsonDoc.isObject())
    {
        ui->textEditLog->append("[" + QDateTime::currentDateTime().toString("hh:mm:ss") + "] Error: " + 
                               QString::fromLocal8Bit("JSON文件格式错误，应为对象格式"));
        return false;
    }

    QJsonObject jsonObject = jsonDoc.object();
    bool success = ProcessJsonData(jsonObject);
    
    if (success)
    {
        ui->textEditLog->append("[" + QDateTime::currentDateTime().toString("hh:mm:ss") + "] Info: " + 
                               QString::fromLocal8Bit("JSON文件处理成功: ") + filename);
    }
    
    return success;
}

/**
 * @brief 处理JSON数据
 * @param jsonData JSON数据对象
 * @return 是否成功处理JSON数据
 */
bool FormRobotPilot::ProcessJsonData(const QJsonObject& jsonData)
{
    // 检查是否有commands数组
    if (jsonData.contains("commands") && jsonData["commands"].isArray())
    {
        QJsonArray commands = jsonData["commands"].toArray();
        
        for (const QJsonValue& commandValue : commands)
        {
            if (commandValue.isObject())
            {
                QJsonObject command = commandValue.toObject();
                if (!ExecuteJsonCommand(command))
                {
                    ui->textEditLog->append("[" + QDateTime::currentDateTime().toString("hh:mm:ss") + "] Error: " + 
                                           QString::fromLocal8Bit("执行JSON命令失败"));
                    return false;
                }
            }
        }
        return true;
    }
    
    // 如果没有commands数组，直接处理单个命令
    return ExecuteJsonCommand(jsonData);
}

/**
 * @brief 执行JSON命令
 * @param command JSON命令对象
 * @return 是否成功执行命令
 */
bool FormRobotPilot::ExecuteJsonCommand(const QJsonObject& command)
{
    if (!command.contains("type") || !command["type"].isString())
    {
        ui->textEditLog->append("[" + QDateTime::currentDateTime().toString("hh:mm:ss") + "] Error: " + 
                               QString::fromLocal8Bit("JSON命令缺少type字段"));
        return false;
    }

    QString commandType = command["type"].toString();
    
    if (commandType == "move")
    {
        // 移动命令
        if (!command.contains("mode") || !command["mode"].isString())
        {
            ui->textEditLog->append("[" + QDateTime::currentDateTime().toString("hh:mm:ss") + "] Error: " + 
                                   QString::fromLocal8Bit("移动命令缺少mode字段"));
            return false;
        }
        
        QString mode = command["mode"].toString();
        QVector<double> values;
        
        // 解析位置参数
        QStringList paramNames = {"x", "y", "z", "rx", "ry", "rz"};
        for (const QString& param : paramNames)
        {
            if (command.contains(param) && command[param].isDouble())
            {
                values.append(command[param].toDouble());
            }
            else
            {
                values.append(0.0);
            }
        }
        
        if (values.size() == 6)
        {
            if (mode == "tool")
            {
                ui->radCartesianTool->click();
                return moveToCartesian(values);
            }
            else if (mode == "reference")
            {
                ui->radCartesianReference->click();
                return moveToCartesian(values);
            }
            else if (mode == "joints")
            {
                ui->radJoints->click();
                return moveToJoints(values);
            }
        }
    }
    else if (commandType == "plan")
    {
        // 规划命令
        QVector<double> values;
        
        // 解析位置参数
        QStringList paramNames = {"x", "y", "z", "rx", "ry", "rz"};
        for (const QString& param : paramNames)
        {
            if (command.contains(param) && command[param].isDouble())
            {
                values.append(command[param].toDouble());
            }
            else
            {
                values.append(0.0);
            }
        }
        
        if (values.size() == 6)
        {
            return PlanningPosition(values);
        }
    }
    else if (commandType == "load_model")
    {
        // 加载模型命令
        if (command.contains("filename") && command["filename"].isString())
        {
            QString filename = command["filename"].toString();
            Item loadedItem = RDK->AddFile(filename);
            return RDK->Valid(loadedItem);
        }
    }
    else if (commandType == "set_pose")
    {
        // 设置模型位姿命令
        if (command.contains("item") && command["item"].isString())
        {
            QString itemName = command["item"].toString();
            Item item = RDK->getItem(itemName);
            
            if (RDK->Valid(item))
            {
                tXYZWPR pose;
                QStringList paramNames = {"x", "y", "z", "rx", "ry", "rz"};
                for (int i = 0; i < 6; ++i)
                {
                    if (command.contains(paramNames[i]) && command[paramNames[i]].isDouble())
                    {
                        pose[i] = command[paramNames[i]].toDouble();
                    }
                    else
                    {
                        pose[i] = 0.0;
                    }
                }
                
                Mat newPose;
                newPose.FromXYZRPW(pose);
                item->setPose(newPose);
                RDK->Render();
                return true;
            }
        }
    }
    else if (commandType == "select_robot")
    {
        // 选择机器人命令
        if (command.contains("name") && command["name"].isString())
        {
            QString robotName = command["name"].toString();
            return SelectRobotByName(robotName);
        }
    }
    
    ui->textEditLog->append("[" + QDateTime::currentDateTime().toString("hh:mm:ss") + "] Error: " + 
                           QString::fromLocal8Bit("未知的JSON命令类型: ") + commandType);
    return false;
}

/**
 * @brief 清空所有目标点
 */
void FormRobotPilot::ClearAllTargets()
{
    // 获取所有目标点
    QVector<Item> targets = m_MapRobotTarget.value(Robot);
    
    // 删除所有目标点
    for (Item target : targets)
    {
        target->Delete();
    }

    m_MapRobotTarget.remove(Robot);
    m_MapTargetPoints.remove(Robot);
    
    if (Robot->Name().split(" ").last() == "L")
    {
        m_pointNumL = 0;
    }
    else
    {
        m_pointNumR = 0;
    }
    
    ui->textEditLog->append("[" + QDateTime::currentDateTime().toString("hh:mm:ss") + "] Info: " + 
                           QString::fromLocal8Bit("所有目标点已清空"));
    RDK->Render();
}

/**
 * @brief 获取当前工具坐标位置
 * @return 工具坐标位置字符串，格式: "x,y,z,rx,ry,rz"
 */
QString FormRobotPilot::GetToolPosition()
{
    if (!Robot)
    {
        return "No robot selected";
    }
    
    // 获取工具位姿
    Robot->Joints();
    Mat toolPose = Robot->PoseTool();
    Mat robotPose = Robot->Pose();
    Mat relativePose = robotPose * toolPose;
    
    // 转换为XYZRPW格式
    tXYZWPR xyzwpr;
    robotPose.ToXYZRPW(xyzwpr);
    
    // 格式化为字符串
    QString result = QString("%1_%2_%3_%4_%5_%6")
                        .arg(xyzwpr[0], 0, 'f', 6)
                        .arg(xyzwpr[1], 0, 'f', 6)
                        .arg(xyzwpr[2], 0, 'f', 6)
                        .arg(xyzwpr[3], 0, 'f', 6)
                        .arg(xyzwpr[4], 0, 'f', 6)
                        .arg(xyzwpr[5], 0, 'f', 6);
    
    ui->textEditLog->append("[" + QDateTime::currentDateTime().toString("hh:mm:ss") + "] Info: " + 
                           QString::fromLocal8Bit("获取工具坐标: ") + result);
    
    return result;
}

/**
 * @brief 通过名称获取机械臂
 * @param name 机械臂名称
 * @return 机械臂Item对象
 */
Item FormRobotPilot::GetRobotByName(const QString& name)
{
    if (name.isEmpty())
    {
        return nullptr;
    }
    
    // 获取所有机器人
    QList<Item> allRobots = RDK->getItemList(IItem::ITEM_TYPE_ROBOT);
    
    // 查找匹配名称的机器人
    for (Item robot : allRobots)
    {
        if (robot->Name() == name)
        {
            return robot;
        }
    }
    
    return nullptr;
}

/**
 * @brief 获取当前关节位置
 * @return 关节位置字符串，格式: "j1,j2,j3,j4,j5,j6"
 */
QString FormRobotPilot::GetJointPositions()
{
    if (!Robot)
    {
        return "No robot selected";
    }
    
    // 获取关节角度
    tJoints joints = Robot->Joints();

    // 格式化为字符串
    QString result = QString("%1_%2_%3_%4_%5_%6")
                        .arg(joints.Data()[0], 0, 'f', 6)
                        .arg(joints.Data()[1], 0, 'f', 6)
                        .arg(joints.Data()[2], 0, 'f', 6)
                        .arg(joints.Data()[3], 0, 'f', 6)
                        .arg(joints.Data()[4], 0, 'f', 6)
                        .arg(joints.Data()[5], 0, 'f', 6);
    
    ui->textEditLog->append("[" + QDateTime::currentDateTime().toString("hh:mm:ss") + "] Info: " + 
                           QString::fromLocal8Bit("获取关节位置: ") + result);
    
    return result;
}

QString FormRobotPilot::GetToolPosition1()
{
    if (!Robot1)
    {
        return "No robot selected";
    }

    // 获取工具位姿
    Robot1->Joints();
    Mat toolPose = Robot1->PoseTool();
    Mat robotPose = Robot1->Pose();
    Mat relativePose = robotPose * toolPose;

    // 转换为XYZRPW格式
    tXYZWPR xyzwpr;
    robotPose.ToXYZRPW(xyzwpr);

    // 格式化为字符串
    QString result = QString("%1_%2_%3_%4_%5_%6")
        .arg(xyzwpr[0], 0, 'f', 6)
        .arg(xyzwpr[1], 0, 'f', 6)
        .arg(xyzwpr[2], 0, 'f', 6)
        .arg(xyzwpr[3], 0, 'f', 6)
        .arg(xyzwpr[4], 0, 'f', 6)
        .arg(xyzwpr[5], 0, 'f', 6);

    ui->textEditLog->append("[" + QDateTime::currentDateTime().toString("hh:mm:ss") + "] Info: " +
        QString::fromLocal8Bit("获取工具坐标: ") + result);

    return result;
}

QString FormRobotPilot::GetJointPositions1()
{
    if (!Robot1)
    {
        return "No robot selected";
    }

    // 获取关节角度
    tJoints joints = Robot1->Joints();

    // 格式化为字符串
    QString result = QString("%1_%2_%3_%4_%5_%6")
        .arg(joints.Data()[0], 0, 'f', 6)
        .arg(joints.Data()[1], 0, 'f', 6)
        .arg(joints.Data()[2], 0, 'f', 6)
        .arg(joints.Data()[3], 0, 'f', 6)
        .arg(joints.Data()[4], 0, 'f', 6)
        .arg(joints.Data()[5], 0, 'f', 6);

    ui->textEditLog->append("[" + QDateTime::currentDateTime().toString("hh:mm:ss") + "] Info: " +
        QString::fromLocal8Bit("获取关节位置: ") + result);

    return result;
}

QString FormRobotPilot::GetToolPosition2()
{
    if (!Robot2)
    {
        return "No robot selected";
    }

    // 获取工具位姿
    Robot2->Joints();
    Mat toolPose = Robot2->PoseTool();
    Mat robotPose = Robot2->Pose();
    Mat relativePose = robotPose * toolPose;

    // 转换为XYZRPW格式
    tXYZWPR xyzwpr;
    robotPose.ToXYZRPW(xyzwpr);

    // 格式化为字符串
    QString result = QString("%1_%2_%3_%4_%5_%6")
        .arg(xyzwpr[0], 0, 'f', 6)
        .arg(xyzwpr[1], 0, 'f', 6)
        .arg(xyzwpr[2], 0, 'f', 6)
        .arg(xyzwpr[3], 0, 'f', 6)
        .arg(xyzwpr[4], 0, 'f', 6)
        .arg(xyzwpr[5], 0, 'f', 6);

    ui->textEditLog->append("[" + QDateTime::currentDateTime().toString("hh:mm:ss") + "] Info: " +
        QString::fromLocal8Bit("获取工具坐标: ") + result);

    return result;
}

QString FormRobotPilot::GetJointPositions2()
{
    if (!Robot2)
    {
        return "No robot selected";
    }

    // 获取关节角度
    tJoints joints = Robot2->Joints();

    // 格式化为字符串
    QString result = QString("%1_%2_%3_%4_%5_%6")
        .arg(joints.Data()[0], 0, 'f', 6)
        .arg(joints.Data()[1], 0, 'f', 6)
        .arg(joints.Data()[2], 0, 'f', 6)
        .arg(joints.Data()[3], 0, 'f', 6)
        .arg(joints.Data()[4], 0, 'f', 6)
        .arg(joints.Data()[5], 0, 'f', 6);

    ui->textEditLog->append("[" + QDateTime::currentDateTime().toString("hh:mm:ss") + "] Info: " +
        QString::fromLocal8Bit("获取关节位置: ") + result);

    return result;
}

/**
 * @brief 设置当前活跃机械臂
 * @param robotName 机械臂名称
 * @return 是否成功设置
 */
bool FormRobotPilot::SetActiveRobot(const QString& robotName)
{
    Item newRobot = GetRobotByName(robotName);
    if (RDK->Valid(newRobot))
    {
        Robot = newRobot;
        ui->lblRobot->setText(QString::fromLocal8Bit("已选择机器人:\n") + robotName);
        RDK->ShowMessage(tr("Robot selected: ") + robotName, false);
        // 更新当前关节/位姿显示
        updateCartesianPosition();
        return true;
    }
    
    RDK->ShowMessage(tr("Robot not found: ") + robotName, false);
    return false;
}

/**
 * @brief 获取所有可用机械臂
 * @return 机械臂名称列表
 */
QStringList FormRobotPilot::GetAvailableRobots()
{
    QStringList robotNames;
    
    // 获取所有机器人
    QList<Item> allRobots = RDK->getItemList(IItem::ITEM_TYPE_ROBOT);
    
    // 提取机器人名称
    for (Item robot : allRobots)
    {
        robotNames.append(robot->Name());
    }
    
    return robotNames;
}

// 新增实现：关节欧氏距离 (tJoints::Dist 等价)
double FormRobotPilot::JointsDistance(const tJoints& j1, const tJoints& j2) {
    int dof = qMin(j1.Length(), j2.Length());  // 取最小 DOF (e.g., 6)
    double dist = 0.0;
    const double* v1 = j1.Values();  // 只读指针
    const double* v2 = j2.Values();
    for (int i = 0; i < dof; ++i) {
        double delta = v1[i] - v2[i];
        dist += delta * delta;  // Σ(ΔJi²)
    }
    return sqrt(dist);  // √(总和)，单位 °
}

// 新增实现：位姿距离 (Mat::Dist 等价)
double FormRobotPilot::PoseDistance(const Mat& p1, const Mat& p2) {
    tXYZWPR xyz1, xyz2;
    p1.ToXYZRPW(xyz1);  // p1 -> XYZRPW (mm, deg)
    p2.ToXYZRPW(xyz2);  // p2 -> XYZRPW

    // 位置距离 (XYZ)
    double posDist = sqrt(
        pow(xyz1[0] - xyz2[0], 2) +  // ΔX
        pow(xyz1[1] - xyz2[1], 2) +  // ΔY
        pow(xyz1[2] - xyz2[2], 2)    // ΔZ
    );

    // 姿态距离 (RPW，度)
    double oriDist = sqrt(
        pow(xyz1[3] - xyz2[3], 2) +  // ΔRx
        pow(xyz1[4] - xyz2[4], 2) +  // ΔRy
        pow(xyz1[5] - xyz2[5], 2)    // ΔRz
    );

    return posDist + oriDist;  // 总距离 (mm + deg)，或加权 (e.g., posDist + oriDist * 0.1)
}

bool FormRobotPilot::ExecuteRobot(QVector<double> &tar)
{
    std::vector<PoseWithIndex> targetPoints = m_MapTargetPoints.value(Robot);
    if (targetPoints.empty())
    {
        RDK->ShowMessage(tr("No target points planned"), false);
        return false;
    }

    int mode = ui->radCartesianReference->isChecked() ? 0 :
        (ui->radCartesianTool->isChecked() ? 1 : 2);
    qDebug() << QString::fromLocal8Bit("读取位置");
    QVector<double> target;
    if (Robot->Name().split(" ").last() == "L")
    {
        target.append(targetPoints[m_pointNumL].pose.x);
        target.append(targetPoints[m_pointNumL].pose.y);
        target.append(targetPoints[m_pointNumL].pose.z);
        target.append(targetPoints[m_pointNumL].pose.rx);
        target.append(targetPoints[m_pointNumL].pose.ry);
        target.append(targetPoints[m_pointNumL].pose.rz);
        m_pointNumL++;
        if (m_pointNumL >= targetPoints.size())
        {
            m_pointNumL = 0;
        }
    }
    else
    {
        target.append(targetPoints[m_pointNumR].pose.x);
        target.append(targetPoints[m_pointNumR].pose.y);
        target.append(targetPoints[m_pointNumR].pose.z);
        target.append(targetPoints[m_pointNumR].pose.rx);
        target.append(targetPoints[m_pointNumR].pose.ry);
        target.append(targetPoints[m_pointNumR].pose.rz);
        m_pointNumR++;
        if (m_pointNumR >= targetPoints.size())
        {
            m_pointNumR = 0;
        }
    }
    tar = target;
    qDebug() << QString::fromLocal8Bit("设置坐标系");
    // 根据记录的坐标系模式设置UI
    switch (mode)
    {
    case 0: ui->radCartesianReference->click(); break;
    case 1: ui->radCartesianTool->click(); break;
    case 2: ui->radJoints->click(); break;
    }
    qDebug() << QString::fromLocal8Bit("执行运动");
    // 执行移动
    bool moveSuccess = false;
    if (mode == 2)
    {
        moveSuccess = moveToJoints(target);
    }
    else
    {
        moveSuccess = moveToCartesian(target);
    }

    return moveSuccess;
}

void FormRobotPilot::GetDistance()
{
    QSettings settings(m_CurrentPath + "/config/config.ini", QSettings::IniFormat);
    settings.beginGroup("Distance");
    m_Distance = settings.value("distance", "600").toInt();
    settings.endGroup();
    settings.beginGroup("EStop");
    m_StopType = settings.value("stoptype", "1").toInt();
    settings.endGroup();
    settings.beginGroup("RadarComMap");
    m_Robot1ComNum = settings.value("ComNumRobot1", "").toString();
    m_Robot2ComNum = settings.value("ComNumRobot2", "").toString();
    settings.endGroup();
    ui->lineEdit_RadarDistance->setText(QString::number(m_Distance));
}

void FormRobotPilot::SetDistance()
{
    QSettings settings(m_CurrentPath + "/config/config.ini", QSettings::IniFormat);
    settings.beginGroup("Distance");
    settings.setValue("distance", m_Distance);
    settings.endGroup();
}

void FormRobotPilot::discoverAndSetupDevices()
{
    const auto ports = QSerialPortInfo::availablePorts();
    const QStringList keywords = { "USB", "CH340", "FTDI", "PL2303", "CP210", "RS485", "Serial" };
    int deviceIndex = 0;
    m_nRadarNum = 0;

    QList<QSerialPortInfo> TmpPort1;
    for (auto It: ports)
    {
        if(It.portName() == m_Robot1ComNum || It.portName() == m_Robot2ComNum)
        {
            TmpPort1.append(It);
        }
    }
    
    for (const QSerialPortInfo& info : TmpPort1)
    {
        QString desc = info.description().toUpper() + info.manufacturer().toUpper();
        bool isLikely = false;
        for (const QString& key : keywords)
            if (desc.contains(key))
            { 
                isLikely = true; 
                break; 
            }

        if (info.vendorIdentifier() == 0x1A86 ||   // CH340
            info.vendorIdentifier() == 0x10C4 ||   // CP2102
            info.vendorIdentifier() == 0x067B)
        {  
            // PL2303
            isLikely = true;
        }

        if (!isLikely) 
            continue;

        RadarDevice* dev = new RadarDevice;
        if(info.portName() == m_Robot1ComNum)
        {
            dev->slaveAddress = 0x01;
            dev->RobotName = QString::fromLocal8Bit("一号机械臂");
        }
        else if (info.portName() == m_Robot2ComNum)
        {
            dev->slaveAddress = 0x02;
            dev->RobotName = QString::fromLocal8Bit("二号机械臂");
        }
        dev->portName = info.portName();
        // dev->slaveAddress = (deviceIndex == 0) ? 0x01 : 0x02;
        dev->addressSet = false;

        // 关联对应机器人（假设 Robot1 和 Robot2 已通过其他方式获取）
        // dev->RobotName = (deviceIndex == 0) ? QString::fromLocal8Bit("一号机械臂") : QString::fromLocal8Bit("二号机械臂");

        dev->serial = new QSerialPort(this);
        dev->serial->setPortName(dev->portName);
        dev->serial->setBaudRate(QSerialPort::Baud115200);
        dev->serial->setDataBits(QSerialPort::Data8);
        dev->serial->setParity(QSerialPort::NoParity);
        dev->serial->setStopBits(QSerialPort::OneStop);

        if (!dev->serial->open(QIODevice::ReadWrite))
        {
            ui->textEditLog->append(QString("打开 %1 失败: %2")
                .arg(dev->portName).arg(dev->serial->errorString()));
            delete dev->serial;
            delete dev;
            continue;
        }

        connect(dev->serial, &QSerialPort::readyRead, this, [this, dev]() { 
        	onDeviceReadyRead(dev);
            });

        dev->timer = new QTimer(this);
        dev->timer->setInterval(500);

        devices.append(dev);

        deviceIndex++;

        ui->textEditLog->append(QString::fromLocal8Bit("发现雷达设备 → %1，准备设置地址 0x%2")
            .arg(dev->portName)
            .arg(dev->slaveAddress, 2, 16, QChar('0')));

        // 延时广播设置地址，避免冲突
        QTimer::singleShot((devices.size() - 1) * 1000, this, [this, dev]() {
            broadcastSetAddress(dev, dev->slaveAddress);
            });
    }

    if (devices.isEmpty())
    {
        ui->textEditLog->append(QString::fromLocal8Bit("未检测到任何雷达设备！"));
    }
    else
    {
        ui->textEditLog->append(QString::fromLocal8Bit("检测到 %1 个雷达设备，正在配置地址...").arg(devices.size()));
    }

    QTimer::singleShot(5000, this, [this]() {
        for (RadarDevice* d : devices)
        {
            connect(d->timer, &QTimer::timeout, this, [this, d]() {
                sendReadAllChannels(d);
                });
            d->timer->start(50);           // 每100ms采集一次
            sendReadAllChannels(d);         // 立即发一次
        }
        });
}

void FormRobotPilot::broadcastSetAddress(RadarDevice* dev, quint8 newAddr)
{
    QByteArray cmd;
    cmd.append(char(0xFF));  // 广播地址
    cmd.append(char(0x06));
    cmd.append(char(0x00));
    cmd.append(char(0x15));
    cmd.append(char(0x00));
    cmd.append(char(newAddr));

    quint16 crc = modbusCRC16(cmd);
    cmd.append(char(crc & 0xFF));
    cmd.append(char(crc >> 8));

    dev->recvBuffer.clear();
    dev->serial->write(cmd);

    ui->textEditLog->append(QString::fromLocal8Bit("[%1] 广播设置地址 → 0x%2")
        .arg(dev->portName).arg(newAddr, 2, 16, QChar('0')));

    QTimer::singleShot(800, this, [this, dev, newAddr]() {
        dev->addressSet = true;  // 广播无返回，强制认为成功
        ui->textEditLog->append(QString::fromLocal8Bit("[%1] 地址设置完成（广播模式）→ 0x%2")
            .arg(dev->portName).arg(newAddr, 2, 16, QChar('0')));

        bool allSet = true;
        for (RadarDevice* d : devices) 
            if (!d->addressSet) 
                allSet = false;

        if (allSet) 
        {
            ui->textEditLog->append(QString::fromLocal8Bit("所有雷达设备配置完成，可开始采集！"));
        }

        });

    m_RadarTimer->start(500);
}

quint16 FormRobotPilot::modbusCRC16(const QByteArray& data)
{
    quint16 crc = 0xFFFF;
    for (int i = 0; i < data.size(); ++i) {
        crc ^= (quint8)data[i];
        for (int j = 0; j < 8; ++j) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

void FormRobotPilot::sendReadAllChannels(RadarDevice* dev)
{
    if (!dev->addressSet) return;

    QByteArray cmd;
    cmd.append(char(dev->slaveAddress));
    cmd.append(char(0x03));
    cmd.append(char(0x00));
    cmd.append(char(0x10));
    cmd.append(char(0x00));
    cmd.append(char(0x04));

    quint16 crc = modbusCRC16(cmd);
    cmd.append(char(crc & 0xFF));
    cmd.append(char(crc >> 8));

    dev->recvBuffer.clear();

    dev->serial->write(cmd);
}

void FormRobotPilot::onDeviceReadyRead(RadarDevice* dev)
{
    dev->recvBuffer += dev->serial->readAll();

    if (dev->recvBuffer.size() < 13) 
        return;

    if ((quint8)dev->recvBuffer[0] != dev->slaveAddress || (quint8)dev->recvBuffer[1] != 0x03 || (quint8)dev->recvBuffer[2] != 0x08)
    {
        dev->recvBuffer.clear();
        return;
    }

    quint16 crcCalc = modbusCRC16(dev->recvBuffer.left(dev->recvBuffer.size() - 2));
    quint16 crcRecv = (quint8)dev->recvBuffer[dev->recvBuffer.size() - 2] | ((quint8)dev->recvBuffer[dev->recvBuffer.size() - 1] << 8);

    if (crcCalc != crcRecv)
    {
        //ui->textEditLog->append(QString::fromLocal8Bit("[%1] CRC校验失败").arg(dev->portName));
        dev->recvBuffer.clear();
        return;
    }

    qint32 ch[4];
    for (int i = 0; i < 4; ++i) 
    {
        ch[i] = ((quint8)dev->recvBuffer[3 + i * 2] << 8) | (quint8)dev->recvBuffer[4 + i * 2];
    }

    // 判断是否触发安全停止
    bool tooClose = false;
    for (int i = 0; i < 4; ++i)
    {
        if (ch[i] > 0 && ch[i] < m_Distance)
        {
            tooClose = true;
            break;
        }
    }

    if(tooClose)
    {
        m_nRadarNum ++;
        ui->textEditLog->append(dev->RobotName + ":"
		+ QString::number(ch[0])
		+ QString::fromLocal8Bit("、")
		+ QString::number(ch[1])
		+ QString::fromLocal8Bit("、")
		+ QString::number(ch[2])
		+ QString::fromLocal8Bit("、")
		+ QString::number(ch[3]) + "\n" + QString::number(m_nRadarNum));

    }

    //if (m_nRadarNum >= 5)
    //{
    //    // 处理雷达触发后事件
    //    if (dev->RobotName.contains(QString::fromLocal8Bit("一号")))
    //    {
    //        // 一号机械臂触发急停
    //        ui->textEditLog->append(QString::fromLocal8Bit("一号机械臂距离过近触发急停"));
    //        if(m_StopType == 2)
    //        {
	   //         m_Radarsocket->write("Light:5;");
    //        }
    //        else
    //        {
    //            m_Radarsocket->write("Light:2;");
    //        }
    //    }
    //    else
    //    {
    //        // 二号机械臂触发急停
    //        ui->textEditLog->append(QString::fromLocal8Bit("二号机械臂距离过近触发急停"));
    //        if (m_StopType == 2)
    //        {
    //            m_Radarsocket->write("Light:5;");
    //        }
    //        else
    //        {
    //            m_Radarsocket->write("Light:1;");
    //        }
    //    }
    //}

    dev->recvBuffer.clear();
}
