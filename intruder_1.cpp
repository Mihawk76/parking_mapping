/**
 * Program to detection motion using simple background substraction
 */
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <string> 
#include <iostream>
#include <sstream>
#include <fstream>

using namespace std;
using namespace cv;
// for log
ofstream data;
std::vector<std::vector<cv::Point> > contours;
ostringstream oss;
RNG rng(12345);
cv::Mat motion;
cv::Mat diffImage;
string grid[500];
Rect rect; // putting a mask
int keyboard; //input from keyboard
Mat frame; //current frame
Point boundary[4]= 
{
  //cv::Point(150,386), cv::Point(23,669), cv::Point(323, 406), cv::Point(197, 682)
  cv::Point(150,386), cv::Point(0,0), cv::Point(0, 0), cv::Point(197, 682)
};
Point boundary_car[2] = { cv::Point(0,0), cv::Point(266,325) };
static void onMouse(int event, int x, int y, int f, void* ){
    cout << x << " " << y << endl;
    //putText(image, "point", Point(x,y), CV_FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255,0,0));
}

void makeGrid(cv::Mat& mat_img)
{
	int stepSize = 65;

	int width = mat_img.size().width;
	int height = mat_img.size().height;

	for (int i = 0; i<height; i += stepSize)
	{
		int retribution = i;
		
		oss << " " << retribution;
		//grid[i] = oss;
		//cout << oss;
    	cv::line(mat_img, cv::Point(0, i), cv::Point(width, i), cv::Scalar(255, 255, 255));
		//cv::putText(mat_img, grid[i], cv::Point(0, i), FONT_HERSHEY_PLAIN, 1 , cv::Scalar(255, 255, 255));
	}

	for (int i = 0; i<width; i += stepSize)
    	cv::line(mat_img, cv::Point(i, 0), cv::Point(i, height), cv::Scalar(255, 255, 255));
}

void intruderAlarm(cv::Mat& bg_frame, cv::Mat& cam_frame)
{
	cv::absdiff(bg_frame, cam_frame, diffImage);
	//cv::threshold(motion, motion, 80, 255, cv::THRESH_BINARY);
	cv::Mat foregroundMask = cv::Mat::zeros(diffImage.rows, diffImage.cols, CV_8UC1);

    float threshold = 30.0f;//30 awalnya
    float dist;

    for(int j=0; j<diffImage.rows; ++j)
        for(int i=0; i<diffImage.cols; ++i)
        {
            cv::Vec3b pix = diffImage.at<cv::Vec3b>(j,i);

            dist = (pix[0]*pix[0] + pix[1]*pix[1] + pix[2]*pix[2]);
            dist = sqrt(dist);

            if(dist>threshold)
            {
                foregroundMask.at<unsigned char>(j,i) = 255;
            }
        }
	cv::imshow("Before", foregroundMask);
	//cv::imshow("Picture", bg_frame);
	//cv::erode(foregroundMask,foregroundMask,cv::Mat());
	//cv::dilate(foregroundMask,foregroundMask,cv::Mat());
	//cv::imshow("After dilate", foregroundMask);
	cv::findContours( foregroundMask, // binary input image 
                               contours, // vector of vectors of points
                               CV_RETR_EXTERNAL, // retrieve only external contours
                               CV_CHAIN_APPROX_NONE); // detect all pixels of each contour
	cv::drawContours( bg_frame, // draw contours here
                                  contours, // draw these contours
                                  -1, // draw all contours
                                  cv::Scalar(255,255,255), // set color
                                  2); // set thickness
	//makeGrid(foregroundMask);
	//Approximate contours to polygons + get bounding rects and circles
  vector<vector<Point> > contours_poly( contours.size() );
	vector<Rect> boundRect( contours.size() );
	vector<Point2f>center( contours.size() );
  vector<float>radius( contours.size() );
	
	//data << "# of contour size: " << contours.size() << endl ;
	for( int i = 0; i < contours.size(); i++ )
  	{
  	// Filtering Blob that is detected to delete false positif 
		if(contourArea(contours[i]) >= 2000)
    {
      approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
      boundRect[i] = boundingRect( Mat(contours_poly[i]) );
    	minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );	
		}
	}	
	
	//cv::imshow("Result", foregroundMask);
	Mat drawing = Mat::zeros( foregroundMask.size(), CV_8UC3 );
  for( int i = 0; i< contours.size(); i++ )
  {
    if (  /*center[i].x > boundary_car[0].x && center[i].x < boundary_car[1].x 
        && center[i].y > boundary_car[0].y && center[i].y < boundary_car[1].y*/1 )
    {
      Scalar color_1 = Scalar(0,0,0);
 		  drawContours( foregroundMask, contours_poly, i, color_1, 1, 8, vector<Vec4i>(), 0, Point() );
 		  rectangle( foregroundMask, boundRect[i].tl(), boundRect[i].br(), color_1, 2, 8, 0 );
 		  circle( foregroundMask, center[i], (int)radius[i], color_1, 2, 8, 0 );
 		  Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
 		  drawContours( drawing, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
 		  rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
 		  circle( drawing, center[i], (int)radius[i], color, 2, 8, 0 );
      cout << center[i] << " this is centre " << endl;
      if ( center[i].x != 0 && center[i].y != 0 )
      {
        data << center[i] << " this is centre " << endl;
      }
    }
  }
  //cv::line(drawing, boundary[0], boundary[3], cv::Scalar(0, 255, 255), 2, 8);
  //cv::line(drawing, cv::Point(0, 370), cv::Point(drawing.size().width, 525), cv::Scalar(0, 0, 255), 2, 8);
	//cv::line(drawing, cv::Point(0, 870), cv::Point(drawing.size().width, 1025), cv::Scalar(0, 0, 255), 2, 8);
  //cv::rectangle(drawing, cv::Point(0, 370), cv::Point(drawing.size().width, 1025), cv::Scalar(0, 255, 255), 2, 8);
	//imshow( "Contours", drawing );
	//imshow( "Final Form", foregroundMask );
}
int main(int argc, char* argv[])
{
		namedWindow("Frame");
    data.clear();
    data.open("data.log",ios::out);
    // Load test images
    cv::Mat a = cv::imread(argv[1]);
		String name = argv[2];
		VideoCapture capture(name);
		if(!capture.isOpened())
		{
			//error in opening the video input
			cerr << "Unable to open video file: " << name << endl;
			exit(EXIT_FAILURE);
  	}
		//read input data. ESC or 'q' for quitting
  	while( (char)keyboard != 'q' && (char)keyboard != 27 )
		{
			//read the current frame
			if(!capture.read(frame)) {
			cerr << "Unable to read next frame." << endl;
			cerr << "Exiting..." << endl;
			exit(EXIT_FAILURE);
		}
    //cv::Mat b = cv::imread(argv[2]);
    rect = Rect(0, 500, 1440, 525); // Rect(x, y, w, h)
		// 655 = 1025−575
		//rect = Rect(0, 500, 268, 400); 
		//cv::Mat a_ROI = a(rect);
    //cv::Mat b_ROI = frame(rect);
    //cv::Mat b_ROI = b(rect);
    //data << " a.size().width " << a.size().width << endl;
    //if (a.empty() || b.empty())
		/*if (a.empty() || frame.empty())
        return -1;*/

    // Start motion detection
    //intruderAlarm(a_ROI, b_ROI);
		intruderAlarm(a, frame);
    // Display result
	
	makeGrid(a);
	cv::line(a, cv::Point(0, 370), cv::Point(a.size().width, 525), cv::Scalar(0, 0, 255), 2, 8);
	cv::line(a, cv::Point(0, 870), cv::Point(a.size().width, 1025), cv::Scalar(0, 0, 255), 2, 8);
	cv::line(a, boundary[0], boundary[3], cv::Scalar(0, 255, 255), 2, 8);
	cv::line(a, boundary[2], boundary[1], cv::Scalar(0, 255, 255), 2, 8);
  	cv::imshow("a", a);
		cv::imshow("Frame", frame);
		//cv::imshow("b", b_ROI);
		//cv::imshow("frame2", b_ROI);
	//setMouseCallback( "a", onMouse, 0 );	
    //cv::imshow("b", b);
		//get the input from the keyboard
  	keyboard = waitKey( 30 );
	}
	//delete capture object
    capture.release();
  //  cv::waitKey(0);

    return 0;
}
// testing
