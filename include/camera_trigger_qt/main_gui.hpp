#pragma once

#include <QMainWindow>


class QPushButton;
class QWidget;
class QLabel;

#include "camera_trigger_qt/ros2node.hpp"

class MainGUI : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainGUI(const std::shared_ptr<Ros2Node> &ros2_node, QWidget* parent = nullptr);
    ~MainGUI() = default;
private:
    void showRgbImage();
    void showIrImage();
    void showOdometry();

    const std::shared_ptr<Ros2Node> m_ros2Node;

    QWidget* m_mainWidget;
    QPushButton* m_cameraTriggerButton;
    QLabel *rgbLabel;
    QLabel *irLabel;
    QLabel *xLabel;
    QLabel *yLabel;
    QLabel *zLabel;
    QLabel *rollLabel;
    QLabel *pitchLabel;
    QLabel *yawLabel;
};