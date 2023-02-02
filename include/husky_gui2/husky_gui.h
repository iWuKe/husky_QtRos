/**
 *
 * Based on https://github.com/xpharry/ros-qt-gui & https://github.com/allenh1/ros-qt-controller & https://github.com/VelazcoJD/QtLogging
 * QProcesses handling is based on: https://www.informit.com/articles/article.aspx?p=1405549&seqNum=5
 *
 * @brief main window
 *
 * @date December 2022
 **/

#ifndef HUSKY_APP_H_
#define HUSKY_APP_H_

#include <QtWidgets>
#include <QMainWindow>
#include <QWidget>
#include <QAction>
#include <QTabWidget>
#include <QListView>
#include <QLineEdit>
#include <QCheckBox>
#include <QPushButton>
#include <QTextEdit>
#include <QSlider>
#include <QtDebug>
#include <QDebug>
#include <QCloseEvent>
#include <QMessageBox>
#include <QPalette>
#include <QIcon>
#include <QMovie>
#include <QMetaType>
#include <QProcess>
#include <iostream>
#include <sstream>
#include "ui_mainwindow.h"
#include "logger.h"
#include "qnode.h"
#include "ros_topics.h"

QT_BEGIN_NAMESPACE
namespace HUSKY_APP { class Logger; class Husky_GUI; }
QT_END_NAMESPACE

namespace HUSKY_APP {
#define PI 3.14159265359

class Husky_GUI : public QMainWindow{
    Q_OBJECT

public:
    Husky_GUI(int argc, char** argv, QWidget *parent = 0);
    ~Husky_GUI();

    //functions
    void ReadSettings(); // Load up qt program settings at startup
    void WriteSettings(); // Save qt program settings when closing
    void closeEvent(QCloseEvent *event); // Overloaded function
    void showNoMasterMessage();
    QStringListModel* loggingModel() { return &logging_model; }

public slots:
    //** Signals connected automatically to GUI controls **//
    void on_actionAbout_triggered();
    void on_button_connect_clicked(bool check );
    void on_checkbox_use_environment_stateChanged(int state);
    void on_button_refresh_clicked();
    void on_button_subscribe_clicked();
    //void on_combobox_pose_currentIndexChanged(int index);
    //void on_combobox_velocity_currentIndexChanged(int index);

    void on_button_rviz_clicked(bool checked);
    void on_button_gazebo_clicked(bool checked);

    //** General usage signals **//  (NOT automatically connected to GUI controls)
    void updateVisualCue(); //ATTENTION: this will need to have a double input in the future
    //void updateVisualCue(double concentration);
    void updatePoseDisplay(double x, double y, double theta);
    void updateCameraFeed(cv::Mat);
    void updateLoggingView();
    void showMessageBox(const QString &text);

signals:
    //** Logging signals **//
    void fatalExcpMsgBox(const QString);
    void loggingUpdated();
    //void gasConcentration(double c);

private slots:
    ///** Manual movement control **///
    void goForward();
    void goBackward();
    void goLeft();
    void goRight();
    void halt();

    ///** Logging **///
    void myLog( const HUSKY_APP::Logger::LogLevel &level, const std::string &msg);
    void myLog2( const HUSKY_APP::Logger::LogLevel &level, const std::string &msg, const std::string &func_name);

    ///** QProcesses (rviz and gazebo) **///
    void onProcStarted();
    void onProcStateChanged(QProcess::ProcessState);
    void onProcReadyReadStandardOutput_map();
    void onProcReadyReadStandardOutput_sim();
    void onProcReadyReadStandardOutput_sim2();
    void onProcReadyReadStandardOutput_rst();
    void onProcReadyReadStandardError_map();
    void onProcReadyReadStandardError_sim();
    void onProcReadyReadStandardError_sim2();
    void onProcReadyReadStandardError_rst();
    void onProcError(QProcess::ProcessError);
    void onProcFinished_map(int exitCode, QProcess::ExitStatus exitStatus);
    void onProcFinished_sim(int exitCode, QProcess::ExitStatus exitStatus);
    void onProcFinished_sim2(int exitCode, QProcess::ExitStatus exitStatus);
    void onProcFinished_rst(int exitCode, QProcess::ExitStatus exitStatus);

    void on_combobox_robot_currentTextChanged(const QString &arg1);

    void on_button_reset_clicked(bool checked);

    void on_button_camera_start_clicked(bool checked);

private:
    Ui::MainWindowDesign ui;
    QNode qnode;
    QStringListModel logging_model;
    QString robot_name;

    QProcess *p_rviz;
    QProcess *kill_rviz;
    QProcess *p_gazb;
    QProcess *p_gazb2;
    QProcess *kill_gazb;
    QProcess *kill_gazb2;
    //qint64 pid_rviz;
    //qint64 pid_gazb;
    //qint64 pid_gazb2;
    QProcess *p_reset;

    // Controls
    QAction *actionAbout_Qt;
    QTabWidget *tab_manager;
    QListView *view_logging;
    QCheckBox *checkbox_remember_settings;
    QCheckBox *checkbox_use_environment;
    QPushButton *button_connect;
    QLineEdit *line_edit_master;
    QLineEdit *line_edit_host;
    QLineEdit *line_edit_topic;

    QPushButton *button_up;
    QPushButton *button_down;
    QPushButton *button_left;
    QPushButton *button_right;
    QPushButton *button_stop;

    QLineEdit *line_edit_xDisplay;
    QLineEdit *line_edit_yDisplay;
    QLineEdit *line_edit_aDisplay;

    QComboBox *combobox_robot;
    QComboBox *combobox_pose;
    QComboBox *combobox_velocity;
    QComboBox *combobox_topics;
    QPushButton *button_refresh;
    QPushButton *button_subscribe;

    QPushButton *button_camera_start;
    QPushButton *button_camera_reset;
    QComboBox *combobox_camera;

    QPushButton *button_rviz;
    QPushButton *button_gazebo;

}; //class Husky_GUI

} //namespace HUSKY_APP

//Q_DECLARE_METATYPE(QProcess::ExitStatus)

#endif // HUSKY_APP_H_
