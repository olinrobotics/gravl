#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <ros/ros.h>

#include "tractor_controller.h"

int main(int argc, char **argv)
{
    QGuiApplication app(argc, argv);

    ros::init(argc, argv, "qtest");
    ros::NodeHandle nh;
    TractorController kubo(&nh);

    QQmlApplicationEngine engine;
    engine.rootContext()->setContextProperty("kubo", &kubo);
    engine.load(QUrl(QStringLiteral("qrc:/main.qml")));

    ros::AsyncSpinner spinner(2);
    spinner.start();

    int result = app.exec();

    spinner.stop();

    return result;
}

#include "main.moc"
