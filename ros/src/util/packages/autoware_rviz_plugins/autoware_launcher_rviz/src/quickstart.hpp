#ifndef AUTOWARE_LAUNCHER_RVIZ_CONTROLLER_HPP
#define AUTOWARE_LAUNCHER_RVIZ_CONTROLLER_HPP

#include <unordered_map>
#include <rviz/panel.h>
#include <QWidget>
#include <QPushButton>
#include <QTcpSocket>

namespace autoware_launcher_rviz {

class QuickStartPanel : public rviz::Panel
{
    Q_OBJECT

    public:

        QuickStartPanel(QWidget* parent = 0);

    private Q_SLOTS:

        void launch_button_toggled(bool checked);

        void server_connected();
        void server_disconnected();
        void server_error();
        void server_ready_read();

    private:

        QTcpSocket* socket;
        std::unordered_map<std::string, QPushButton*> buttons;
};

}

#endif
