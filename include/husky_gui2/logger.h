/**
 *
 * Based on https://github.com/VelazcoJD/QtLogging
 *
 * @brief Logging
 *
 * @date November 2022
 *
 **/

#ifndef LOGGER_H
#define LOGGER_H

#include <QDebug>
#include <QFile>
#include <QHash>
#include <QStringListModel>
#include <QMetaType>
#include <QMetaObject>
#include <stdlib.h>
#include <string>
#include <iostream>
#include "assert.h"

#include <ros/ros.h>

namespace HUSKY_APP {

class Logger : public QObject
{
    Q_OBJECT

public:

    enum LogLevel {
             Debug,
             Info,
             Warn,
             Error,
             Fatal,
             Excp
     };   //Q_ENUM(LogLevel)

    /// @brief Initializes the logger.
    static void init();

    /// @brief Cleans up the logger.
    static void clean();

    /// @brief The function which handles the logging of text.
    static void messageOutput(QtMsgType type, const QMessageLogContext& context,
        const QString& msg);

private:
    /// @brief The file object where logs are written to.
    static QFile* logFile;

    /// @brief Whether the logger has being initialized.
    static bool isInit;

    /// @brief The different type of contexts.
    static QHash<QtMsgType, QString> contextNames;

};//end class Logger

}//end namespace

Q_DECLARE_METATYPE(HUSKY_APP::Logger::LogLevel)
Q_DECLARE_METATYPE(std::string)

#endif // LOGGER_H

