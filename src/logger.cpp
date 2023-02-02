/**
 *
 * Based on https://github.com/VelazcoJD/QtLogging
 *
 * @brief Logging
 *
 * @date November 2022
 *
 **/

#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QHash>
#include <QObject>
#include <sstream>
#include "./include/husky_gui2/logger.h"

namespace HUSKY_APP{

QFile* Logger::logFile = Q_NULLPTR;
bool Logger::isInit = false;
QHash<QtMsgType, QString> Logger::contextNames = {
    {QtMsgType::QtDebugMsg,		" Debug  "},
    {QtMsgType::QtInfoMsg,		"  Info  "},
    {QtMsgType::QtWarningMsg,	"Warning "},
    {QtMsgType::QtCriticalMsg,	"Critical"},
    {QtMsgType::QtFatalMsg,		" Fatal  "},
    {QtMsgType::QtFatalMsg,		"Exception"}
};

void Logger::init() {

    if (isInit) {
        return;
    }

    // Create log file
    logFile = new QFile;
    logFile->setFileName("My_QtROS_Gui_Log.log");
    logFile->open(QIODevice::Append | QIODevice::Text);

    // Redirect logs to messageOutput
    qInstallMessageHandler(Logger::messageOutput);

    // Clear file contents
    logFile->resize(0);

    Logger::isInit = true;
}// end constructor

void Logger::clean() {
    if (logFile != Q_NULLPTR) {
        logFile->close();
        delete logFile;
    }
}//end destructor -> closes logFile and cleans resources

void Logger::messageOutput(QtMsgType type, const QMessageLogContext& context, const QString& msg) {

    QString log = QObject::tr("%1 | %2 | %3 | %4 | %5 | %6\n").
        arg(QDateTime::currentDateTime().toString("dd-MM-yyyy hh:mm:ss.zzz")).
        arg(Logger::contextNames.value(type)).
        arg(context.line).
        arg(QString(context.file).
            section('/', -1)).			// File name without file path  - for Linux
            //section('\\', -1)).			// File name without file path - for Windows
        arg(QString(context.function).
            section('(', -2, -2).		// Function name only
            section(' ', -1).
            section(':', -1)).
        arg(msg);

    logFile->write(log.toLocal8Bit());
    logFile->flush();
}//end messageOutput -> outputs the logging message into the log file

}//end namespace

