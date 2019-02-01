#include "quickstart.hpp"
#include <QApplication>
#include <QDesktopWidget>
#include <QGridLayout>

#include <iostream>
using namespace std;

namespace {

QPushButton* create_push_button(QString title)
{
    auto button = new QPushButton(title);
    button->setEnabled(false);
    button->setCheckable(true);
    button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    return button;
}

}

namespace autoware_launcher_rviz {

QuickStartPanel::QuickStartPanel(QWidget* parent) : rviz::Panel(parent)
{
    QRect screen = QApplication::desktop()->screenGeometry();
    int font_size = min(screen.width(), screen.height()) / 100;
    setStyleSheet(QString("font-size: %1px;").arg(font_size));

    auto layout = new QGridLayout();
    setLayout(layout);

    std::string rootpath = "root/";
    std::vector<const char*> nodenames = {"localization", "detection", "prediction", "decision", "mission", "motion"};
    for(size_t i = 0; i < nodenames.size(); ++i)
    {
        auto button = create_push_button(nodenames[i]);
        buttons[rootpath + nodenames[i]] = button;
        layout->addWidget(button, i/3, i%3);
        connect(button, &QPushButton::toggled, this, &QuickStartPanel::launch_button_toggled);
    }

    socket = new QTcpSocket(this);
    connect(socket, &QTcpSocket::connected,    this, &QuickStartPanel::server_connected   );
    connect(socket, &QTcpSocket::disconnected, this, &QuickStartPanel::server_disconnected);
    connect(socket, &QTcpSocket::readyRead,    this, &QuickStartPanel::server_ready_read  );
    connect(socket, static_cast<void(QTcpSocket::*)(QAbstractSocket::SocketError)>(&QTcpSocket::error), this, &QuickStartPanel::server_error);
    socket->connectToHost("localhost", 33136);
}

void QuickStartPanel::launch_button_toggled(bool checked)
{
    auto button = static_cast<QPushButton*>(sender());
    QString json = R"({"command":"%1", "path":"root/%2"})";
    json = json.arg(checked ? "launch" : "terminate").arg(button->text());
    cout << json.toStdString() << endl;
    socket->write(json.toUtf8().append('\0'));
}

void QuickStartPanel::server_connected()
{
    cout << "connected" << endl;
    for(const auto& pair : buttons)
    {
        pair.second->setEnabled(true);
    }
}

void QuickStartPanel::server_disconnected()
{
    cout << "disconnected" << endl;
    for(const auto& pair : buttons)
    {
        pair.second->setEnabled(false);
    }
}

void QuickStartPanel::server_error()
{
    cout << "error" << endl;
    cout << socket->errorString().toStdString() << endl;
}

void QuickStartPanel::server_ready_read()
{
    cout << "ready_read" << endl;
    cout << socket->readAll().toStdString() << endl;
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(autoware_launcher_rviz::QuickStartPanel, rviz::Panel)
