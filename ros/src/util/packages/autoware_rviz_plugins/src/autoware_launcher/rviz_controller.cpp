#include "rviz_controller.hpp"
#include <QGridLayout>
#include <QPushButton>

#include <iostream>
#include <vector>
using namespace std;

namespace {

QPushButton* create_push_button(QString title)
{
    auto button = new QPushButton(title);
    button->setCheckable(true);
    button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    return button;
}

}

namespace autoware_rviz_plugins {

RvizController::RvizController(QWidget* parent) : rviz::Panel(parent)
{
    auto layout = new QGridLayout();
    setLayout(layout);

    vector<const char*> titles = {"map", "localization"};
    for(size_t i = 0; i < titles.size(); ++i)
    {
        auto button = create_push_button(titles[i]);
        layout->addWidget(button, 0, i);
        connect(button, &QPushButton::toggled, this, &RvizController::launch_button_toggled);
    }

    socket = new QTcpSocket(this);
    connect(socket, &QTcpSocket::connected,    this, &RvizController::server_connected   );
    connect(socket, &QTcpSocket::disconnected, this, &RvizController::server_disconnected);
    connect(socket, &QTcpSocket::readyRead,    this, &RvizController::server_ready_read  );
    connect(socket, static_cast<void(QTcpSocket::*)(QAbstractSocket::SocketError)>(&QTcpSocket::error), this, &RvizController::server_error);
    socket->connectToHost("localhost", 33136);
}

void RvizController::launch_button_toggled(bool checked)
{
    auto button = static_cast<QPushButton*>(sender());
    if(checked)
    {
        QString json = R"({"command":"launch", "path":["root","%1"]})";
        json = json.arg(button->text());
        cout << json.toStdString() << endl;
        socket->write(json.toUtf8().append('\0'));
    }
}

void RvizController::server_connected()
{
    cout << "connected" << endl;
}

void RvizController::server_disconnected()
{
    cout << "disconnected" << endl;
}

void RvizController::server_error()
{
    cout << "error" << endl;
    cout << socket->errorString().toStdString() << endl;
}

void RvizController::server_ready_read()
{
    cout << "ready_read" << endl;
    cout << socket->readAll().toStdString() << endl;
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(autoware_rviz_plugins::RvizController, rviz::Panel)