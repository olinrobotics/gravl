#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <ros/ros.h>

#include "tractor_controller.h"

int main(int argc, char **argv)
{
    // Create application object
    QGuiApplication app(argc, argv);

    // Initialize ROS stuff
    ros::init(argc, argv, "qtest");
    ros::NodeHandle nh;
    TractorController kubo(&nh);

    // Set up QML stuff
    QQmlApplicationEngine engine;
    engine.rootContext()->setContextProperty("kubo", &kubo);
    engine.load(QUrl(QStringLiteral("qrc:/main.qml")));

    // Create ROS Async spinner b/c app.exec is blocking
    ros::AsyncSpinner spinner(2);
    spinner.start();

    // Start the app
    int result = app.exec();

    // Stop spinner
    spinner.stop();

    return result;
}

#include "main.moc"
