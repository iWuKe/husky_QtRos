/**
 *
 * Based on https://github.com/xpharry/ros-qt-gui
 *
 * @brief Qt based gui.
 *
 * @date October 2022
 **/
/*****************************************************************************
** Includes
*****************************************************************************/
#include <QApplication>
#include <QtWidgets>
#include <QMainWindow>
#include <QString>
#include <QLocale>
#include <QTranslator>
#include <QMetaType>
#include <iostream>
#include <ui_mainwindow.h>
#include "./include/husky_gui2/husky_gui.h"
#include "./include/husky_gui2/logger.h"

/*****************************************************************************
** Main
*****************************************************************************/


int main(int argc, char** argv){

//    if( !ros::isInitialized() )
//    {
//      ros::init( argc, argv, "husky_gui_main");
//    }
//    ros::start(); // explicitly needed since our nodehandle is going out of scope.
//    ros::NodeHandle n;

    QApplication app(argc, argv);
    qRegisterMetaType<HUSKY_APP::Logger::LogLevel>();
    qRegisterMetaType<std::string>();
    qRegisterMetaType<cv::Mat>();

    //Translator stuff (not necessary for creating mainwindow and qnode)
    QTranslator translator;
    const QStringList uiLanguages = QLocale::system().uiLanguages();
    for (const QString &locale : uiLanguages) {
        const QString baseName = "husky_app2_" + QLocale(locale).name();
        if (translator.load(":/i18n/" + baseName)) {
            app.installTranslator(&translator);
            break;
        }
    }

    HUSKY_APP::Husky_GUI w(argc,argv);
    HUSKY_APP::Logger::init();
    w.show();
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();

    HUSKY_APP::Logger::clean();
    return result;
}
