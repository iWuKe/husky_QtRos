/**
 *
 * Based on https://github.com/xpharry/ros-qt-gui & https://github.com/allenh1/ros-qt-controller & https://github.com/VelazcoJD/QtLogging
 * QProcesses handling is based on: https://www.informit.com/articles/article.aspx?p=1405549&seqNum=5
 *
 * @brief main window
 *
 * @date December 2022
 **/

/*****************************************************************************
** Includes
*****************************************************************************/
#include <QMainWindow>
#include <QtWidgets>
//#include <QGuiApplication>
//#include <QQmlApplicationEngine>
#include <QObject>
#include <QProcess>
#include <QLineEdit>
#include <QCheckBox>
#include <QPushButton>
#include <QTextEdit>
#include <QSettings>
#include <QtDebug>
#include <QDebug>
#include <QMessageBox>
#include <QProcess>
#include <QStringList>
#include <QMovie>
#include <QTimer>
/*#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"*/
#include <ros/ros.h>
#include <ros/master.h>
#include <qtimer.h>
#include <iostream>
#include <ui_mainwindow.h>
#include "./include/husky_gui2/husky_gui.h"
#include "./include/husky_gui2/logger.h"
#include "./include/husky_gui2/public_params.h"

/*****************************************************************************
** Global variables
*****************************************************************************/

///** the following thress char arrays are based on the corresponding QProcess
/// enum members and help print the string descriptions instead of the int **///
const char *myProcessState[] =
{
    "QProcess::NotRunning",
    "QProcess::Starting",
    "QProcess::Running"
};

const char *myProcessError [] =
{
    "QProcess::FailedToStart",
    "QProcess::Crashed",
    "QProcess::Timedout",
    "QProcess::ReadError",
    "QProcess::WriteError",
    "QProcess::UnknownError"
};

/*****************************************************************************
** Namespaces
*****************************************************************************/
QT_BEGIN_NAMESPACE
namespace HUSKY_APP {
using namespace Qt;}
QT_END_NAMESPACE

namespace HUSKY_APP {

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

Husky_GUI::Husky_GUI(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , qnode(argc, argv)
    , p_rviz() //QProcesses must be declared as members, otherwise ui cannot access them - segmentation error occurs
    , p_gazb() //            <<         <<
    , kill_rviz()
    , kill_gazb()
{
    ui.setupUi(this);

    /********************
    ** Initial actions **
    ** *****************/
    //std::cout << "OpenCV: " << cv::getBuildInformation().c_str() << std::endl;
    PubPrms::getMyIpAddress();
    ReadSettings();
    //setWindowIcon(QIcon(":/images/icon.png"));
    ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    //const QHostAddress &localhost = QHostAddress(QHostAddress::LocalHost);

    /**********************
    ** Auto Start Signal **
    **********************/
    if ( ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked(true);
    }

    /***********************
    ** Setup gui controls **
    ** ********************/

    ///* Setup !!temporary!! visual cue
    /*QMovie* movie = new QMovie(":/images/visual_cue_green.gif");
    if (!movie->isValid())
    {
        std::cout << "Something went wrong with the movie :(" << std::endl;
    }
    ui.label_visual_cue->setMovie(movie);
    movie->start();*/

    /*QTimer *gas_Timer = new QTimer(this);
    QObject::connect(gas_Timer, SIGNAL(timeout()), this, SLOT(updateVisualCue()));
    gas_Timer->start(2000);*/
    ///* end of temporarily dummy gas concentration  ATTENTION

    ///* Setup default camera feed
    /*QMovie* movie_camera = new QMovie(":/images/wcamerafeed.gif");
    if (!movie_camera->isValid())
    {
        std::cout << "Something went wrong with the camera_img :(" << std::endl;
    }
    ui.label_camerafeed->setMovie(movie_camera);
    movie_camera->start();*/

    //QTimer *rs_camera_Timer = new QTimer(this);
    //QObject::connect(rs_camera_Timer, SIGNAL(timeout()), this, SLOT(updateRealSenseCamera()));
    //rs_camera_Timer->rs_camera_Timer(500);
    ///* end camera feed

    //ui.line_edit_master->setText(_DFLT_MASTER_URI_);
    //ui.line_edit_host->setText(_DFLT_ROS_IP_);
    //ui.line_edit_topic->setText(_DFLT_VEL_TOPIC_);
    ui.combobox_robot->addItems(HUSKY_APP::PubPrms::_ROBOTS_);
    ui.combobox_robot->setCurrentIndex(2);//this set the default value of robot to 'Simulator_Husky'
    ui.combobox_pose->addItems(HUSKY_APP::PubPrms::_DFLT_POSE_TOPICS_);
    ui.combobox_pose->setCurrentIndex(2);//this set the default pose topic to 'Simulator_Husky'
    ui.combobox_velocity->addItems(HUSKY_APP::PubPrms::_DFLT_VEL_TOPICS_);
    ui.combobox_velocity->setCurrentIndex(2);//this set the default velocity topic to 'Simulator_Husky'

    /******************************
    ** Connect signals and slots **
    ** ***************************/

    // related to manual movement
    QObject::connect(ui.button_up,    SIGNAL(clicked(bool)), this, SLOT(goForward()));
    QObject::connect(ui.button_left,  SIGNAL(clicked(bool)), this, SLOT(goLeft()));
    QObject::connect(ui.button_right, SIGNAL(clicked(bool)), this, SLOT(goRight()));
    QObject::connect(ui.button_down,  SIGNAL(clicked(bool)), this, SLOT(goBackward()));
    QObject::connect(ui.button_stop,  SIGNAL(clicked(bool)), this, SLOT(halt()));

    //related to QNode qthread
    QObject::connect(&qnode, SIGNAL(newPose(double,double,double)),
                     this, SLOT(updatePoseDisplay(double, double, double)));
    QObject::connect(&qnode, SIGNAL(newImage(cv::Mat)),this, SLOT(updateCameraFeed(cv::Mat)));
    QObject::connect(&qnode, SIGNAL(sendLog2Gui(const HUSKY_APP::Logger::LogLevel&,
                                                const std::string&)),
                     this, SLOT(myLog(const HUSKY_APP::Logger::LogLevel&, const std::string&)));
    /*QObject::connect(&qnode, SIGNAL(sendLog2Gui2(const HUSKY_APP::Logger::LogLevel&,
                                                 const std::string&, const std::string&)),
                     this, SLOT(myLog2(const HUSKY_APP::Logger::LogLevel&, const std::string&,
                                     const std::string&)));*/
    //QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

    //**related to QProcesses (rviz and gazebo)**//
    // These signals are attached to slots when the QProcesses are created
    // (otherwise the sender is null and the connection fail)

    //miscellaneous
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    /******************
    ** Setup Logging **
    ******************/
    ui.view_logging->setModel(this->loggingModel());
    QObject::connect(this, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));
    QObject::connect(this, SIGNAL(fatalExcpMsgBox(const QString)),
                     this, SLOT(showMessageBox(const QString)));
}// end constructor of  mainwindow

Husky_GUI::~Husky_GUI() {
    std::cout << "Qt/ROS GUI shutdown." << std::endl;
    qInfo() << "[INFO]" << "Qt/ROS GUI shutdown.";
    try {

        if (&qnode)
            delete &qnode;

        if (&p_rviz)
        {
            p_rviz->terminate();
            QProcess* kill_rviz = new QProcess(this);
            QString str = "rosnode kill /rviz && kill -SIGINT " + QString::number(p_rviz->processId() + 1);
            kill_rviz->start(str);
            kill_rviz->waitForStarted();
            kill_rviz = nullptr;
        }

        if (&p_gazb)
        {
            p_gazb->terminate();
            QProcess* kill_gazb = new QProcess(this);
            QString str = "kill -SIGINT " + QString::number(p_gazb->processId() + 1);
            //QString str = "rosnode kill /gazebo && rosnode kill /gazebo_gui && kill -SIGINT " + QString::number(p_gazb->processId() + 1);
            kill_gazb->start(str);
            kill_gazb->waitForStarted();
            kill_gazb = nullptr;
        }

    } catch (char *excp){
        //std::cout << "Exception caught: " << excp << std::endl;
        std::string s(excp);
        myLog(Logger::Excp, s);
        qDebug() << "[EXCP]" << excp;
    }
}//end destructor of mainwindow

void Husky_GUI::closeEvent(QCloseEvent *event) {
    try {
        WriteSettings();
        //qInfo() << "Proceeding to close the GUI." ;
        QMainWindow::closeEvent(event);
    }
    catch (char *excp){
        //std::cout << "Exception caught: " << excp << std::endl;
        std::string s(excp);
        myLog(Logger::Excp, s);
        qDebug() << "[EXCP]" << excp;
    }
}//end closing event

/*****************************************************************************
                             **  GUI Controls Slots **
*****************************************************************************/

void Husky_GUI::on_combobox_robot_currentTextChanged(const QString &arg1)
{
    Husky_GUI::robot_name = arg1;
    //Set the robot index for the correct topics to be used for velocity and pose
    PubPrms::_DFLT_CB_INDEX_ = PubPrms::robot_index.value(Husky_GUI::robot_name);
    ui.combobox_velocity->setCurrentText(PubPrms::_DFLT_VEL_TOPICS_[PubPrms::_DFLT_CB_INDEX_]);
    ui.combobox_pose->setCurrentText(PubPrms::_DFLT_POSE_TOPICS_[PubPrms::_DFLT_CB_INDEX_]);
}

//void Husky_GUI::on_combobox_pose_currentIndexChanged(int index) {

//}

//void Husky_GUI::on_combobox_velocity_currentIndexChanged(int index) {

//}

void Husky_GUI::on_button_rviz_clicked(bool checked) {
    //char* path = getenv("PATH");
    //QString l(path);
    //std::cout << l.toStdString() << std::endl;
    //std::cout << QProcess::workingDirectory().toStdString() << std::endl;
    //QString l("/opt/ros/noetic/");
    try {
        if (p_rviz)
            delete p_rviz;
        myLog(Logger::Info, std::string("Mapping/rviz starting"));
        qInfo() << "[INFO]" << "Mapping/rviz starting";
        if (true) //(!p_rviz)
        {
            p_rviz = new QProcess(this);
            //p_rviz->QProcess::setWorkingDirectory(l);
            //std::cout << p_rviz->workingDirectory().toStdString() << std::endl;
            QObject::connect(p_rviz, SIGNAL(started()), this,
                             SLOT(onProcStarted()), Qt::DirectConnection);
            QObject::connect(p_rviz, SIGNAL(stateChanged(QProcess::ProcessState)), this,
                             SLOT(onProcStateChanged(QProcess::ProcessState)),  Qt::DirectConnection);
            //QObject::connect(p_rviz, SIGNAL(readyReadStandardOutput()), this,
                             //SLOT(onProcReadyReadStandardOutput_map()), Qt::DirectConnection); // this one outputs too much (not necessarily usefull) stuff - keep for debugging only)
            QObject::connect(p_rviz, SIGNAL(readyReadStandardError()), this,
                             SLOT(onProcReadyReadStandardError_map()), Qt::DirectConnection);
            QObject::connect(p_rviz, SIGNAL(finished(int, QProcess::ExitStatus)),
                             this, SLOT(onProcFinished_map(int,QProcess::ExitStatus)), Qt::DirectConnection);
            QObject::connect(p_rviz, SIGNAL(errorOccurred(QProcess::ProcessError)), this,
                             SLOT(onProcError(QProcess::ProcessError)), Qt::DirectConnection);

            QString exec1 = "xterm";
            QString exec2 = "sh";
            QString master = "export ROS_MASTER_URI=" + ui.line_edit_master->text() + ";";
            QString source = "source ~/catkin_ws/devel/setup.bash;";
            QString s1 = source + master + "roslaunch husky_viz view_robot.launch"; //option 1
            QString s2 = source + master +"rosrun rviz rviz"; //option 2
            QString s3 = source + master +"rosrun rviz rviz -f velodyne"; //option 3
            QString s4 = source + master +"rosrun rviz rviz -f odom"; //option 4
            QString s5 = source + master +"rosrun rqt_gui rqt_gui"; //option 5

            if (robot_name == "Simulator_Husky")
                p_rviz->start(exec1, QStringList() << "-hold" << "-e" << s3);
            else
                p_rviz->start(exec1, QStringList() << "-hold" << "-e" << s3);

            if(!p_rviz->waitForStarted())
            {
                //std::cout << "Rviz failed to start." << std::endl;
                myLog(Logger::Info, std::string("Mapping/rviz failed to start"));
                qInfo() << "[INFO]" << "Mapping/rviz failed to start";
            }
            ///** use startDetach when you do not need to keep track of the QProcess
            /// which lives even after the termination/kill of the QProcess
            /// In that case, in order to kill the qprocess a 'kill -KILL <pid>' command
            /// must be send to the terminal **///
            /*p_rviz->startDetached(QString::fromUtf8("sh"),
                                  QStringList()<<"-c"<< s.c_str(),"", &pid_rviz);
            pid_rviz++; //the corresponding pid in the terminal is higher  by 1
            myLog( Logger::Debug, std::string("Mapping/rviz started with pid = ") + std::to_string(pid_rviz) );
            qInfo() << "[INFO]" << "Mapping/rviz started with pid = " << pid_rviz;*/
            //myLog( Logger::Debug, std::string("Mapping/Rviz process started"));
            //ui.button_rviz->setEnabled(false);
        }
        //p_rviz->waitForReadyRead();
        //QString p_stdout = p_rviz->readAll();
        //QString p_stderr = p_rviz->readAllStandardError();
    } catch (char *excp){
        //std::cout << "Exception caught: " << excp << std::endl;
        std::string s(excp);
        myLog(Logger::Excp, s);
        qDebug() << "[EXCP]" << excp;
    }
}

void Husky_GUI::on_button_gazebo_clicked(bool checked)
{
    try {
        myLog(Logger::Info, std::string("Simulator/gazebo starting"));
        qInfo() << "[INFO]" << "Simulator/gazebo starting";
        if (true) //(!p_gazb)
        {
             // Start gazebo world with husky robot
            p_gazb = new QProcess(this);
            QObject::connect(p_gazb, SIGNAL(started()), this,
                             SLOT(onProcStarted()), Qt::DirectConnection);
            QObject::connect(p_gazb, SIGNAL(stateChanged(QProcess::ProcessState)), this,
                             SLOT(onProcStateChanged(QProcess::ProcessState)),  Qt::DirectConnection);
            //QObject::connect(p_gazb, SIGNAL(readyReadStandardOutput()), this,
                             //SLOT(onProcReadyReadStandardOutput_sim()), Qt::DirectConnection); // this one outputs too much (not necessarily usefull) stuff - keep for debugging only)
            QObject::connect(p_gazb, SIGNAL(readyReadStandardError()), this,
                             SLOT(onProcReadyReadStandardError_sim()), Qt::DirectConnection);
            QObject::connect(p_gazb, SIGNAL(finished(int, QProcess::ExitStatus)),
                             this, SLOT(onProcFinished_sim(int,QProcess::ExitStatus)), Qt::DirectConnection);
            QObject::connect(p_gazb, SIGNAL(errorOccurred(QProcess::ProcessError)), this,
                             SLOT(onProcError(QProcess::ProcessError)), Qt::DirectConnection);//*/

            QString exec1 = "xterm";
            QString exec2 = "sh";
            QString master = "export ROS_MASTER_URI=" + ui.line_edit_master->text() + ";";
            QString source = "source ~/catkin_ws/devel/setup.bash;";
            QString s1 = source + master + "roslaunch husky_gazebo husky_empty_world.launch"; //option 1
            QString s2 = source + master + "roslaunch husky_gazebo husky_playpen.launch"; //option 2
            QString s3 = source + master + "roslaunch husky_gazebo husky_mine7.launch"; //option 3

            p_gazb->start(exec1, QStringList() << "-hold" << "-e" << s1);

            if(!p_gazb->waitForStarted())
            {
                myLog(Logger::Info, std::string("Simulator/gazebo failed to start"));
                qInfo() << "[INFO]" << "Simulator/gazebo failed to start";
            }

            // Start rviz world with husky robot
            ui.button_rviz->animateClick();
        }
    } catch (char *excp){
        std::string s(excp);
        myLog(Logger::Excp, s);
        qDebug() << "[EXCP]" << excp;
    }
}

/// @brief This triggers whenever the button is clicked, regardless of whether it
/// is already checked or not.
 void Husky_GUI::on_button_connect_clicked(bool check) {
    if ( ui.checkbox_use_environment->isChecked() ) {
        if ( !qnode.init() ) {
            showNoMasterMessage();
        } else {
            //ui.button_connect->setEnabled(false);
            ui.button_rviz->setEnabled(true);
            ui.button_gazebo->setEnabled(true);
            ui.button_refresh->setEnabled(true);
            ui.button_subscribe->setEnabled(true);
            ui.button_refresh->animateClick();
        }
    } else {
        if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
                    ui.line_edit_host->text().toStdString(),
                    ui.combobox_velocity->currentText().toStdString(),
                    ui.combobox_pose->currentText().toStdString())) {
            showNoMasterMessage();
        } else {
            //ui.button_connect->setEnabled(false);
            ui.button_rviz->setEnabled(true);
            ui.button_gazebo->setEnabled(true);
            ui.button_refresh->setEnabled(true);
            ui.button_subscribe->setEnabled(true);
            //ui.line_edit_master->setReadOnly(true);
            //ui.line_edit_host->setReadOnly(true);
            //ui.line_edit_topic->setReadOnly(true);
            ui.button_refresh->animateClick();
        }
    }
}

void Husky_GUI::on_checkbox_use_environment_stateChanged(int state) {
    bool enabled;
    if ( state == 0 )
        enabled = true;
    else
        enabled = false;
    ui.line_edit_master->setEnabled(enabled);
    ui.line_edit_host->setEnabled(enabled);
}

void Husky_GUI::on_button_refresh_clicked() {
    try {
        QString targetV("vel");
        QString targetP("pose");
        QString targetC("camera");
        QString targetVt("geometry_msg/Twist");
        QString targetPt("nav_msgs/Odometry");
        QString targetCt("sensor_msgs/Image");

        // clear existing lists
        ui.combobox_velocity->clear();
        ui.combobox_pose->clear();
        ui.combobox_topics->clear();
        ui.combobox_camera->clear();

        //Retrieve ROS system state (i.e., publishers, subscribers, services)
        PubPrms::topics_subs.clear();
        PubPrms::topics_pubs.clear();
        PubPrms::service_providers.clear();
        bool newSysSt = TopicsHandle::getSystemState(PubPrms::topics_pubs, PubPrms::topics_subs, PubPrms::service_providers);

        //Retrieve ROS master topics and types
        PubPrms::topics_type.clear();
        //bool newTopics = TopicsHandle::getRosTopics(PubPrms::topics_type); //this retrieves ALL topics (not published ones too!)
        bool newTopics = TopicsHandle::getPublishedRosTopics(PubPrms::topics_type); //this retrieves only published topics
        //PubPrms::topics_type = TopicsHandle::getRosTopics(); //same as previous - debugging only
        if (!newTopics) {
            std::string s = "Failed to get the published ROS topics";
            myLog(Logger::Error, s);
            qDebug() << "[ERROR]" << s.c_str();
        }
        else {
            for (int i = 0; i < PubPrms::topics_type.size(); ++i){
                QString t_n = PubPrms::topics_type[i].name.c_str();
                QString t_t = PubPrms::topics_type[i].datatype.c_str();

                ui.combobox_topics->addItem(t_n);
                /*if (t_n.contains(targetV,Qt::CaseInsensitive) || t_t.contains(targetVt,Qt::CaseSensitive))
                    ui.combobox_velocity->addItem(t_n);
                if (t_n.contains(targetP,Qt::CaseInsensitive) || t_t.contains(targetPt,Qt::CaseSensitive))
                    ui.combobox_pose->addItem(t_n);
                if (t_n.contains(targetC,Qt::CaseInsensitive) || t_t.contains(targetCt,Qt::CaseSensitive))
                    ui.combobox_camera->addItem(t_n);*/
                if (t_n.contains(targetV,Qt::CaseInsensitive) || t_t.contains(targetVt,Qt::CaseInsensitive))
                    ui.combobox_velocity->addItem(t_n);
                if (t_n.contains(targetP,Qt::CaseInsensitive) || t_t.contains(targetPt,Qt::CaseInsensitive))
                    ui.combobox_pose->addItem(t_n);
                if (t_t.contains(targetCt,Qt::CaseInsensitive))
                    ui.combobox_camera->addItem(t_n);
            }
            std::string s = "Got " + std::to_string(PubPrms::topics_type.size()) + " published ROS topics";
            myLog(Logger::Info, s);
            qDebug() << "[INFO]" << s.c_str();
        }

    } catch (char *excp){
        //std::cout << "Exception caught: " << excp << std::endl;
        std::string s(excp);
        myLog(Logger::Excp, s);
        qDebug() << "[EXCP]" << excp;
    }
}

void Husky_GUI::on_button_subscribe_clicked() {
    try {
        int index = ui.combobox_topics->currentIndex();
        ros::master::TopicInfo selected_topic = PubPrms::topics_type[index];
        TopicsHandle::publish2Topic(*qnode.nh_, selected_topic);
    } catch (char *excp){
        //std::cout << "Exception caught: " << excp << std::endl;
        std::string s(excp);
        myLog(Logger::Excp, s);
        qDebug() << "[EXCP]" << excp;
    }
}

void Husky_GUI::on_button_reset_clicked(bool checked) {

    try {
        myLog(Logger::Info, std::string("Resetting"));
        qInfo() << "[INFO]" << "Resetting";

        //delete &qnode;
        if(ros::isStarted()) {
          ros::shutdown();
          ros::waitForShutdown();
        }

        if (true) //if (!p_reset)
        {
            // Reset process
            p_reset = new QProcess(this);
            QObject::connect(p_reset, SIGNAL(started()), this,
                             SLOT(onProcStarted()), Qt::DirectConnection);
            QObject::connect(p_reset, SIGNAL(stateChanged(QProcess::ProcessState)), this,
                             SLOT(onProcStateChanged(QProcess::ProcessState)),  Qt::DirectConnection);
            //QObject::connect(p_reset, SIGNAL(readyReadStandardOutput()), this,
                             //SLOT(onProcReadyReadStandardOutput_rst()), Qt::DirectConnection); // this one outputs too much (not necessarily usefull) stuff - keep for debugging only)
            QObject::connect(p_reset, SIGNAL(readyReadStandardError()), this,
                             SLOT(onProcReadyReadStandardError_rst()), Qt::DirectConnection);
            QObject::connect(p_reset, SIGNAL(finished(int, QProcess::ExitStatus)),
                             this, SLOT(onProcFinished_rst(int,QProcess::ExitStatus)), Qt::DirectConnection);
            QObject::connect(p_reset, SIGNAL(errorOccurred(QProcess::ProcessError)), this,
                             SLOT(onProcError(QProcess::ProcessError)), Qt::DirectConnection);//*/
            std::string s = "yes | rosnode cleanup purge";
            p_reset->start("sh", QStringList()<<"-c"<< s.c_str());
            p_reset->waitForStarted();

            if(!p_reset->waitForStarted())
            {
                myLog(Logger::Info, std::string("Reset failed"));
                qInfo() << "[INFO]" << "Reset failed";
            }
            myLog( Logger::Info, std::string("Reset signal sent") );
            qInfo() << "[INFO]" << "Reset signal sent";
        }
    } catch (char *excp){
        std::string s(excp);
        myLog(Logger::Excp, s);
        qDebug() << "[EXCP]" << excp;
    }
}

void Husky_GUI::on_button_camera_start_clicked(bool checked)
{
    try {
        qnode.initImageSubscriber("camera_listener_node", ui.line_edit_master->text().toStdString(),
                              ui.line_edit_host->text().toStdString(),
                              ui.combobox_camera->currentText().toStdString());
        //ros::Subscriber camera_listener = qnode.nh_->subscribe(camera_topic,10, &QNode::imageCallback, &qnode);
    } catch (char *excp){
        //std::cout << "Exception caught: " << excp << std::endl;
        std::string s(excp);
        myLog(Logger::Excp, s);
        qDebug() << "[EXCP]" << excp;
    }
}

/*******************************************************************************
                       ** QProcesses (rviz and gazebo) **
********************************************************************************/
void Husky_GUI::onProcStarted()
{
    myLog(Logger::Debug, std::string("QProcess Started"));
    qDebug() << "[DEBUG]" << "QProcess Started";
}

void Husky_GUI::onProcStateChanged(QProcess::ProcessState newState)
{
    std::ostringstream ss;
    ss << myProcessState[newState];
    myLog(Logger::Debug, std::string("QProcess has new state: ") + ss.str());
    qDebug() << "[DEBUG]" << "QProcess has new state: " << newState;
}

void Husky_GUI::onProcReadyReadStandardOutput_map()
{
    QString line(p_rviz->readAllStandardOutput());
    myLog(Logger::Debug, std::string("Rviz all std output: ") + line.toStdString());
    qDebug() << "[DEBUG]" << "Rviz all std output: " << line;
    //_cout << "QProcess out: " << line << endl << flush;
}

void Husky_GUI::onProcReadyReadStandardOutput_sim()
{
    QString line(p_gazb->readAllStandardOutput());
    myLog(Logger::Debug, std::string("Gazebo all std output: ") + line.toStdString());
    qDebug() << "[DEBUG]" << "Gazebo all std output: " << line;
    //_cout << "QProcess out: " << line << endl << flush;
}

void Husky_GUI::onProcReadyReadStandardOutput_sim2()
{
    QString line(p_gazb2->readAllStandardOutput());
    myLog(Logger::Debug, std::string("Gazebo all std output: ") + line.toStdString());
    qDebug() << "[DEBUG]" << "Gazebo all std output: " << line;
    //_cout << "QProcess out: " << line << endl << flush;
}

void Husky_GUI::onProcReadyReadStandardOutput_rst()
{
    QString line(p_reset->readAllStandardOutput());
    myLog(Logger::Debug, std::string("Reset all std output: ") + line.toStdString());
    qDebug() << "[DEBUG]" << "Reset all std output: " << line;
    //_cout << "QProcess out: " << line << endl << flush;
}

void Husky_GUI::onProcReadyReadStandardError_map()
{
    QString line(p_rviz->readAllStandardOutput());
    myLog(Logger::Debug, std::string("Rviz std ERROR: ") + line.toStdString());
    qDebug() << "[DEBUG]" << "Rviz std ERROR: " << line;
    //_cerr << "Shutdown ERROR: " << line << endl << flush;
}

void Husky_GUI::onProcReadyReadStandardError_sim()
{
    QString line(p_gazb->readAllStandardOutput());
    myLog(Logger::Debug, std::string("Gazebo std ERROR: ") + line.toStdString());
    qDebug() << "[DEBUG]" << "Gazebo std ERROR: " << line;
    //_cerr << "Shutdown ERROR: " << line << endl << flush;
}

void Husky_GUI::onProcReadyReadStandardError_sim2()
{
    QString line(p_gazb2->readAllStandardOutput());
    myLog(Logger::Debug, std::string("Gazebo std ERROR: ") + line.toStdString());
    qDebug() << "[DEBUG]" << "Gazebo std ERROR: " << line;
    //_cerr << "Shutdown ERROR: " << line << endl << flush;
}

void Husky_GUI::onProcReadyReadStandardError_rst()
{
    QString line(p_reset->readAllStandardOutput());
    myLog(Logger::Debug, std::string("Reset std ERROR: ") + line.toStdString());
    qDebug() << "[DEBUG]" << "Reset std ERROR: " << line;
    //_cerr << "Shutdown ERROR: " << line << endl << flush;
}

void Husky_GUI::onProcError(QProcess::ProcessError error)
{
    std::ostringstream ss;
    ss << myProcessError[error];
    myLog(Logger::Debug, std::string("QProcess error: ") + ss.str());
    qDebug() << "[DEBUG]" << "QProcess error: " << error;
}

void Husky_GUI::onProcFinished_map(int exitCode, QProcess::ExitStatus exitStatus)
{
    std::stringstream ss;
    ss << exitStatus;
    if (exitStatus == QProcess::CrashExit) {
        myLog(Logger::Debug, std::string("Rviz  crashed"));
        qDebug() << "[DEBUG]" << "Rviz  crashed";
        //std::cout << "Rviz  crashed" << std::endl;
    } else if (exitCode != 0) {
        myLog(Logger::Debug, std::string("Rviz  failed"));
        qDebug() << "[DEBUG]" << "Rviz  failed";
        //std::cout << "Rviz  failed" << std::endl;
    } else {
        myLog(Logger::Debug, std::string("Rviz  finished with exitCode: ") + ss.str());
        qDebug() << "[DEBUG]" << "Rviz  finished with exitCode: " << exitStatus;
        //std::cout << "Rviz  finished with exitCode: " << exitCode << std::endl;
    }
    ui.button_rviz->setEnabled(true);
    //p_rviz->terminate();
    //p_rviz = nullptr;
}

void Husky_GUI::onProcFinished_sim(int exitCode, QProcess::ExitStatus exitStatus)
{
    std::stringstream ss;
    ss << exitStatus;
    if (exitStatus == QProcess::CrashExit) {
        myLog(Logger::Debug, std::string("Gazebo  crashed"));
        qDebug() << "[DEBUG]" << "Gazebo  crashed";
        //std::cout << "Gazebo  crashed" << std::endl;
    } else if (exitCode != 0) {
        myLog(Logger::Debug, std::string("Gazebo  failed"));
        qDebug() << "[DEBUG]" << "Gazebo  failed";
        //std::cout << "Gazebo  failed" << std::endl;
    } else {
        myLog(Logger::Debug, std::string("Gazebo  finished with exitCode: ") + ss.str());
        qDebug() << "[DEBUG]" << "Gazebo  finished with exitCode: " << exitStatus;
        //std::cout << "Gazebo  finished with exitCode: " << exitCode << std::endl;
    }
    ui.button_gazebo->setEnabled(true);
    //p_gazb->terminate();
    //p_gazb = nullptr;
}

void Husky_GUI::onProcFinished_sim2(int exitCode, QProcess::ExitStatus exitStatus)
{
    std::stringstream ss;
    ss << exitStatus;
    if (exitStatus == QProcess::CrashExit) {
        myLog(Logger::Debug, std::string("Simulator rviz crashed"));
        qDebug() << "[DEBUG]" << "Simulator rviz crashed";
        //std::cout << "Simulator rviz crashed" << std::endl;
    } else if (exitCode != 0) {
        myLog(Logger::Debug, std::string("Simulator rviz failed"));
        qDebug() << "[DEBUG]" << "Simulator rviz failed";
        //std::cout << "Simulator rviz failed" << std::endl;
    } else {
        myLog(Logger::Debug, std::string("Simulator rviz finished with exitCode: ") + ss.str());
        qDebug() << "[DEBUG]" << "Simulator rviz finished with exitCode: " << exitStatus;
        //std::cout << "Simulator rviz finished with exitCode: " << exitCode << std::endl;
    }
    ui.button_gazebo->setEnabled(true);
    //p_gazb->terminate();
    //p_gazb = nullptr;
}

void Husky_GUI::onProcFinished_rst(int exitCode, QProcess::ExitStatus exitStatus)
{
    std::stringstream ss;
    ss << exitStatus;
    if (exitStatus == QProcess::CrashExit) {
        myLog(Logger::Debug, std::string("Reset process crashed"));
        qDebug() << "[DEBUG]" << "Reset process crashed";
        //std::cout << "Reset process crashed" << std::endl;
    } else if (exitCode != 0) {
        myLog(Logger::Debug, std::string("Reset process failed"));
        qDebug() << "[DEBUG]" << "Reset process failed";
        //std::cout << "Reset process failed" << std::endl;
    } else {
        myLog(Logger::Debug, std::string("Reset process finished with exitCode: ") + ss.str());
        qDebug() << "[DEBUG]" << "Reset process finished with exitCode: " << exitStatus;
        //std::cout << "Reset process finished with exitCode: " << exitCode << std::endl;
    }
}

/*****************************************************************************
                             ** Movement Control **
*****************************************************************************/
void Husky_GUI::goForward(){qnode.SetSpeed(0.75, 0);}
void Husky_GUI::goBackward(){qnode.SetSpeed(-0.75, 0);}
void Husky_GUI::goRight(){qnode.SetSpeed(0, -PI / 6.0);}
void Husky_GUI::goLeft(){qnode.SetSpeed(0, PI / 6.0);}
void Husky_GUI::halt(){qnode.SetSpeed(0, 0); }

void Husky_GUI::updateVisualCue()
{
    ///* Remove the first two lines after fixing ROS connection
    double gas = PubPrms::a[ (PubPrms::gas_i / PubPrms::CYCLE) & 1 ? PubPrms::CYCLE - PubPrms::gas_i % PubPrms::CYCLE : PubPrms::gas_i % PubPrms::CYCLE ];
    PubPrms::gas_i++;
    ///* see above ATTENTION

    //ui.label_visual_cue->clear();
    QMovie* movie = new QMovie(":/images/visual_cue_red.gif");
    if (gas <= 6.0)
    {
        delete movie;
        QMovie* movie = new QMovie(":/images/visual_cue_green.gif");
    }
    if (!movie->isValid())
    {
        std::cout << "Something went wrong with the movie :(" << std::endl;
    }
    ui.label_visual_cue->clear();
    ui.label_visual_cue->setMovie(movie);
    ui.label_visual_cue->setScaledContents(true);
    movie->start();
}//update the visual cue

void Husky_GUI::updatePoseDisplay(double x, double y, double theta)
{
    QString xPose, yPose, aPose;
    xPose.setNum(x);
    yPose.setNum(y);
    aPose.setNum(theta);

    ui.line_edit_xDisplay->setText(xPose);
    ui.line_edit_yDisplay->setText(yPose);
    ui.line_edit_aDisplay->setText(aPose);
}//update the pose text displays in the gui

void Husky_GUI::updateCameraFeed(cv::Mat dest)
{
    QImage image1= QImage((uchar*) dest.data, dest.cols, dest.rows, dest.step, QImage::Format_RGB888);

    //show Qimage using QLabel
    ui.label_camerafeed->setPixmap(QPixmap::fromImage(image1));

//    QMovie* movie_camera = new QMovie("camera_image.jpeg");
//    if (!movie_camera->isValid())
//    {
//        std::cout << "Something went wrong with the camera_img :(" << std::endl;
//    }
//    ui.label_camerafeed->setMovie(movie_camera);
//    movie_camera->start();
}//update the camera feed in the gui

/*****************************************************************************
                                ** Menu **
*****************************************************************************/
void Husky_GUI::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
                             ** Configuration **
*****************************************************************************/

void Husky_GUI::ReadSettings() {
    try{
        QSettings settings("Qt/ROS Package", "husky_gui");
        restoreGeometry(settings.value("geometry").toByteArray());
        restoreState(settings.value("windowState").toByteArray());
        //QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
        //QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
        //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
        QString master_url = settings.value("master_url",QString(PubPrms::_DFLT_MASTER_URI_)).toString();
        QString host_url = settings.value("host_url", QString(PubPrms::_DFLT_ROS_IP_)).toString();
        QString vel_topic = settings.value("velocity_topic", QString(PubPrms::_DFLT_VEL_TOPICS_[PubPrms::_DFLT_CB_INDEX_])).toString();
        QString pose_topic = settings.value("pose_topic", QString(PubPrms::_DFLT_POSE_TOPICS_[PubPrms::_DFLT_CB_INDEX_])).toString();
        ui.line_edit_master->setText(master_url);
        ui.line_edit_host->setText(host_url);
        ui.combobox_velocity->setCurrentIndex(PubPrms::_DFLT_CB_INDEX_);
        ui.combobox_pose->setCurrentIndex(PubPrms::_DFLT_CB_INDEX_);
        bool remember = settings.value("remember_settings", false).toBool();
        ui.checkbox_remember_settings->setChecked(remember);
        bool checked = settings.value("use_environment_variables", false).toBool();
        ui.checkbox_use_environment->setChecked(checked);
        if ( checked ) {
            ui.line_edit_master->setEnabled(false);
            ui.line_edit_host->setEnabled(false);
            //ui.combobox_velocity->setEnabled(false);
            //ui.combobox_pose->setEnabled(false);
        }
    }
    catch(char *excp){
        std::cout << "Exception caught: " << excp << std::endl;
        std::string s(excp);
        myLog(Logger::Excp, s);
        qDebug() << "[EXCP]" << excp;
    }
}//read last stored settings

void Husky_GUI::WriteSettings() {
    try{
        QSettings settings("Qt-Ros Package", "HUSKY_APP");
        settings.setValue("master_url",ui.line_edit_master->text());
        settings.setValue("host_url",ui.line_edit_host->text());
        settings.setValue("velocity_topic",ui.combobox_velocity->currentText());
        settings.setValue("pose_topic",ui.combobox_pose->currentText());
        settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
        settings.setValue("geometry", saveGeometry());
        settings.setValue("windowState", saveState());
        settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));
    }
    catch(char *excp){
        std::cout << "Exception caught: " << excp << std::endl;
        std::string s(excp);
        myLog(Logger::Excp, s);
        qDebug() << "[EXCP]" << excp;
    }

}//write latest settings

/*****************************************************************************
** Logging
*****************************************************************************/

void Husky_GUI::showNoMasterMessage() {
    QMessageBox msgBox;
    msgBox.setText("Couldn't find the ros master.");
    msgBox.exec();
    close();
}//end showNoMasterMessage

void Husky_GUI::showMessageBox(const QString &text) {
    QMessageBox msgBox;
    msgBox.setText(text);
    msgBox.exec();
    close();
}//end showMessageBox - for fatal errors and exceptions

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void Husky_GUI::updateLoggingView() {
        ui.view_logging->scrollToBottom();
}

void Husky_GUI::myLog( const HUSKY_APP::Logger::LogLevel &level, const std::string &msg) {
    logging_model.insertRows(logging_model.rowCount(),1);
    std::stringstream logging_model_msg;
    switch ( level ) {
        case(Logger::Debug) : {
                ROS_DEBUG_STREAM(msg);
                logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
                //qDebug() << "[DEBUG] " << msg.c_str();
                //emit onDebugS(msg.c_str());
                break;
        }
        case(Logger::Info) : {
                ROS_INFO_STREAM(msg);
                logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
                //qInfo() << "[INFO] " << msg.c_str();
                //emit onInfoS(msg.c_str());
                break;
        }
        case(Logger::Warn) : {
                ROS_WARN_STREAM(msg);
                logging_model_msg << "[WARN] [" << ros::Time::now() << "]: " << msg;
                //qWarning() << "[WARN] " << msg.c_str();
                //emit onWarningS(msg.c_str());
                break;
        }
        case(Logger::Error) : {
                ROS_ERROR_STREAM(msg);
                logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
                //qCritical() << "[CRITICAL/ERROR] " << msg.c_str();
                //emit onCriticalS(msg.c_str());
                break;
        }
        case(Logger::Fatal) : {
                ROS_FATAL_STREAM(msg);
                logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
                qFatal("%s", msg.c_str());
                //emit onFatalS(msg.c_str());
                break;
        }
        case(Logger::Excp) : { //ATTENTION: Re-think about having exceptions as Fatal messages
                ROS_FATAL_STREAM(msg);
                logging_model_msg << "[EXCEPTION/FATAL] [" << ros::Time::now() << "]: " << msg;
                //qFatal("%s", msg.c_str());
                //emit onFatalS(msg.c_str());
                //emit fatalExcpMsgBox(msg.c_str());
                break;
        }
    }
    QVariant new_row(QString(logging_model_msg.str().c_str()));
    logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
    emit loggingUpdated(); // used to readjust the scrollbar
} //end ROS logging

void Husky_GUI::myLog2(const HUSKY_APP::Logger::LogLevel &level,
                     const std::string &msg,
                     const std::string &function_name) {
    logging_model.insertRows(logging_model.rowCount(),1);
    std::stringstream logging_model_msg;
    std::string fnct = function_name.substr(function_name.find("::") + 2,function_name.find("::"));
    //std::string fnct = function_name.substr(function_name.rfind("::") + 2,function_name.rfind("::"));
    std::string msg2("[@FUNC-> " + fnct + "]: " + msg);
    switch ( level ) {
        case(Logger::Debug) : {
                ROS_DEBUG_STREAM(msg2);
                logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
                //emit onDebugS(msg2.c_str());
                break;
        }
        case(Logger::Info) : {
                ROS_INFO_STREAM(msg2);
                logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
                //emit onInfoS(msg2.c_str());
                break;
        }
        case(Logger::Warn) : {
                ROS_WARN_STREAM(msg2);
                logging_model_msg << "[WARN] [" << ros::Time::now() << "]: " << msg;
                //emit onWarningS(msg2.c_str());
                break;
        }
        case(Logger::Error) : {
                ROS_ERROR_STREAM(msg2);
                logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
                //emit onCriticalS(msg2.c_str());
                break;
        }
        case(Logger::Fatal) : {
                ROS_FATAL_STREAM(msg2);
                logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
                //emit onFatalS(msg2.c_str());
                break;
        }
        case(Logger::Excp) : { //ATTENTION: Re-think about having exceptions as Fatal messages
                ROS_FATAL_STREAM(msg2);
                logging_model_msg << "[EXCEPTION/FATAL] [" << ros::Time::now() << "]: " << msg;
                //emit onFatalS(msg2.c_str());
                //emit fatalExcpMsgBox(msg.c_str());
                break;
        }
    }
    QVariant new_row(QString(logging_model_msg.str().c_str()));
    logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
    emit loggingUpdated(); // used to readjust the scrollbar
} //end ROS logging - function and line are parsed here (and not from Logger::messageOutput()

}  // namespace HUSKY_APP
