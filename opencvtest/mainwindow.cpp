#include "mainwindow.h"
#include "ui_mainwindow.h"

#include<string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    //namedWindow(("My Image"));


    Mat image, houghImage, grayImage;
    Mat outputImage;
    string path = "C:\\Users\\Leon\\Documents\\autoBilder\\";
    ui->setupUi(this);

    //bild einlesen
    image = cv::imread(path + "auto3bearb.jpeg", 1);
    Rect myROI(0, 350, 850, 300);
    //Rect myROI(0, 250, 850, 400);
    Mat croppedImage = image(myROI);
    Mat croppedNormalImage = image(myROI);

     Canny(croppedImage, houghImage, 220, 440, 3);
    //bild grau färben
    cv::cvtColor(houghImage, grayImage, COLOR_GRAY2BGR);

    vector<Vec4i> lines;
      HoughLinesP(houghImage, lines, 2, CV_PI/180, 50, 10, 30 );
      for( size_t i = 0; i < lines.size(); i++ )
      {
        Vec4i l = lines[i];
        line( grayImage, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
      }


    imshow("Image", grayImage);
    imshow("normales Bild", croppedNormalImage);

}

MainWindow::~MainWindow()
{
    delete ui;
}
