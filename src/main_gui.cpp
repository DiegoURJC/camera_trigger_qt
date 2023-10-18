#include "camera_trigger_qt/main_gui.hpp"

#include <QPushButton>
#include <QBoxLayout>
#include <QTimer>
#include <QLabel>

MainGUI::MainGUI(const std::shared_ptr<Ros2Node>& ros2_node, QWidget* parent)
  : QMainWindow(parent)
  , m_ros2Node(ros2_node)
{
  m_mainWidget = new QWidget(this);

  QVBoxLayout* mainLayout = new QVBoxLayout;

  QGridLayout *odometryLayout = new QGridLayout;

  xLabel = new QLabel("x: N/A");
  yLabel = new QLabel("y: N/A");
  zLabel = new QLabel("z: N/A");
  rollLabel = new QLabel("roll: N/A");
  pitchLabel = new QLabel("pitch: N/A");
  yawLabel = new QLabel("yaw: N/A");

  odometryLayout->addWidget(xLabel, 0, 0);
  odometryLayout->addWidget(yLabel, 0, 1);
  odometryLayout->addWidget(zLabel, 0, 2);
  odometryLayout->addWidget(rollLabel, 1, 0);
  odometryLayout->addWidget(pitchLabel, 1, 1);
  odometryLayout->addWidget(yawLabel, 1, 2);


  mainLayout->setSpacing(20);
  mainLayout->setMargin(20);

  rgbLabel = new QLabel;
  irLabel = new QLabel;
  m_cameraTriggerButton = new QPushButton("Capture RGB and IR Images");

  (void)connect(m_cameraTriggerButton, &QPushButton::clicked, this, [this]() {m_ros2Node->captureImages();});

  mainLayout->addWidget(rgbLabel);
  mainLayout->addWidget(irLabel);

  mainLayout->addLayout(odometryLayout);

  mainLayout->addWidget(m_cameraTriggerButton);

  m_mainWidget->setLayout(mainLayout);

  m_mainWidget->adjustSize();

  setCentralWidget(m_mainWidget);

  QTimer *rgbTimer = new QTimer(this);
  QTimer *irTimer = new QTimer(this);
  QTimer *odomTimer = new QTimer(this);

  (void)connect(rgbTimer, &QTimer::timeout, this, &MainGUI::showRgbImage);
  (void)connect(irTimer, &QTimer::timeout, this, &MainGUI::showIrImage);
  (void)connect(odomTimer, &QTimer::timeout, this, &MainGUI::showOdometry);

  rgbTimer->start(300);
  irTimer->start(300);
  odomTimer->start(100);
}

void MainGUI::showRgbImage()
{
  cv_bridge::CvImagePtr rgbPtr = m_ros2Node->getRgbImage();

  if(rgbPtr != nullptr)
  {
    std::cout << "CAPTURING RGB IMAGE..." << std::endl;
    cv::Mat rgbImage = rgbPtr->image;  

    QImage rgbImageQt((uchar*)rgbImage.data, rgbImage.cols, rgbImage.rows, rgbImage.step, QImage::Format_RGB888);
    rgbLabel->setPixmap(QPixmap::fromImage(rgbImageQt).scaled(500, 250));
  }
}

void MainGUI::showIrImage()
{
  cv_bridge::CvImagePtr irPtr = m_ros2Node->getIrImage();

  if(irPtr != nullptr)
  {
    std::cout << "CAPTURING IR IMAGE..." << std::endl;
    cv::Mat irImage = irPtr->image;

    QImage irImageQt((uchar*)irImage.data, irImage.cols, irImage.rows, irImage.step, QImage::Format_Grayscale8);
    irLabel->setPixmap(QPixmap::fromImage(irImageQt).scaled(500, 250));
  }
}

void MainGUI::showOdometry()
{
  const std::array<double, 3> position = m_ros2Node->getPosition();
  const std::array<double, 3> odom = m_ros2Node->getOdometry();

  const QString xText = QString("X: %1").arg(position[0]);
  const QString yText = QString("Y: %1").arg(position[1]);
  const QString zText = QString("Z: %1").arg(position[2]);
  const QString rollText = QString("Roll: %1 deg").arg((odom[0]*180.0)/3.1415);
  const QString pitchText = QString("Pitch: %1 deg").arg((odom[1]*180.0)/3.1415);
  const QString yawText = QString("Yaw: %1 deg").arg((odom[2]*180.0)/3.1415);

  xLabel->setText(xText);
  yLabel->setText(yText);
  zLabel->setText(zText);

  rollLabel->setText(rollText);
  pitchLabel->setText(pitchText);
  yawLabel->setText(yawText);


}