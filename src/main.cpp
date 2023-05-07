#include <QApplication>

#include "camera_trigger_qt/ros2node.hpp"
#include "camera_trigger_qt/main_gui.hpp"


static void siginthandler(int /*param*/)
{
    QApplication::quit();
}

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);

    rclcpp::init(argc, argv);

    auto ros2_node = std::make_shared<Ros2Node>();
    auto gui_app = std::make_shared<MainGUI>(ros2_node);

    // (void)connect(gui_app, &MainGUI::windowClosed, nullptr, &shutdownApp);

    app.processEvents();
    gui_app->show();

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(ros2_node);

    while ((rclcpp::ok()) && (gui_app->isVisible() == true))
    {
        exec.spin_some();
        app.processEvents();
    }

    signal(SIGINT, siginthandler);

    exec.remove_node(ros2_node);
    rclcpp::shutdown();
}