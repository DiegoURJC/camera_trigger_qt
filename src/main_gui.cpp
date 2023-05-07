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

  QGridLayout* mainLayout = new QGridLayout;

  mainLayout->setSpacing(20);
  mainLayout->setMargin(20);

  rgbLabel = new QLabel;
  irLabel = new QLabel;
  m_cameraTriggerButton = new QPushButton("Capture RGB & IR Images");

  // (void)connect(publish_button, &QPushButton::clicked, this, &MainGUI::publish_button_clicked);
  (void)connect(m_cameraTriggerButton, &QPushButton::clicked, this, [this]() {m_ros2Node->captureImages();});

  mainLayout->addWidget(rgbLabel, 0, 0);
  mainLayout->addWidget(irLabel, 0, 1);

  mainLayout->addWidget(m_cameraTriggerButton, 1, 0, -1, -1, Qt::AlignHCenter);

  m_mainWidget->setLayout(mainLayout);

  m_mainWidget->adjustSize();

  setCentralWidget(m_mainWidget);

  QTimer *rgbTimer = new QTimer(this);
  QTimer *irTimer = new QTimer(this);

  (void)connect(rgbTimer, &QTimer::timeout, this, &MainGUI::showRgbImage);
  (void)connect(irTimer, &QTimer::timeout, this, &MainGUI::showIrImage);

  rgbTimer->start(300);
  irTimer->start(300);
}

void MainGUI::showRgbImage()
{
  cv_bridge::CvImagePtr rgbPtr = m_ros2Node->getRgbImage();

  if(rgbPtr != nullptr)
  {
    std::cout << "CAPTURING RGB IMAGE..." << std::endl;
    cv::Mat rgbImage = rgbPtr->image;  

    QImage rgbImageQt((uchar*)rgbImage.data, rgbImage.cols, rgbImage.rows, rgbImage.step, QImage::Format_RGB888);
    rgbLabel->setPixmap(QPixmap::fromImage(rgbImageQt));
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
    irLabel->setPixmap(QPixmap::fromImage(irImageQt));
  }
}