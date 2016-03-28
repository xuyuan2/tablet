#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/video/video.hpp>
#include <iostream>
#include <sstream>
#include <raspicam.h>

using namespace std;

int main()
{
	// The most two importent threshold
	int valThreshold = 150;
	int componentsThr = 150;

	// There two are the same, but due to the API we can only get double as return type
	double numComponents = 0;
	int numArea;


	// frameColor could be also used as frameGray and frameBinary 
	cv::Mat frameConnected;
    
    raspicam::RaspiCam Camera;
    //Open camera 
    cout<<"Opening Camera..."<<endl;
    
    if ( !Camera.open()) 
    {
		cerr<<"Error opening camera"<<endl;return -1;
	}
	
    //wait a while until camera stabilizes
    
    
    

	while(1)
	{
		// Pass the video frame into frameColor!
			
		Camera.grab();
		
		//allocate memory
		unsigned char *data=new unsigned char[  Camera.getImageTypeSize ( raspicam::RASPICAM_FORMAT_RGB )];
		
		//extract the image in rgb format
		Camera.retrieve ( data,raspicam::RASPICAM_FORMAT_RGB );//get camera image
		
		cv::Mat frameColor(Camera.getHeight(), Camera.getWidth(), CV_8UC3, data);

		// Downsampling
		cv::resize(frameColor, frameColor, cv::Size(frameColor.rows/4, frameColor.cols/4));
		
		// Show the image
		cv::namedWindow("fingerTrack", cv::WINDOW_NORMAL);
		imshow("fingerTrack", frameColor);
		if (cv::waitKey(30) >= 0) 
		{
			cv::waitKey(0);
		}

		// Change frameColor to frameGray
		cv::cvtColor(frameColor, frameColor, CV_RGB2GRAY); 
		int width = frameColor.cols;
		int height = frameColor.rows;

		// Generate the mask and do the dilation
		cv::threshold(frameColor, frameColor, valThreshold, 1, CV_THRESH_BINARY);
		cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT , cv::Size(3,7));
		cv::morphologyEx(frameColor, frameColor, cv::MORPH_OPEN, element,cv::Point(-1,-1),1);

		// Connect the bright area
		cv::connectedComponents(frameColor, frameConnected, 4);
		cv::minMaxLoc(frameConnected, NULL, &numComponents);

		// numArea means how many areas are there
		numArea = (int)numComponents;

		// To get the histogram for the connected area
		uchar* frameColorPtr = (uchar*)frameColor.data;
		int* countPtr = (int*)malloc(sizeof(int) * (numArea + 1));
		memset(countPtr, 0, sizeof(int) * (numArea + 1));
		unsigned int* labelPtr = (unsigned int*)frameConnected.data;
		for (int r = 0; r < frameConnected.rows; r++)
		{
			unsigned int* labelLinePtr = &labelPtr[r * width];
			for (int c = 0; c < frameConnected.cols; c++)
			{
				countPtr[labelLinePtr[c]]++;
			}
		}

		// If the histogram is smaller than the threshold, then all the pixel fall in this bin would be erased
		for (int r = 0; r < height; r++)
		{
			unsigned int* labelLinePtr = &labelPtr[r * width];
			uchar* frameColorLinePtr = &frameColorPtr[r * width];
			for (int c = 0; c < width; c++)
			{
				if (countPtr[labelLinePtr[c]] < componentsThr)
				{
					frameColorLinePtr[c] = 0;
				}
			}
		}

		// Init vector
		vector<cv::Point2i> pointsPos(numArea + 1);
		for (int i = 0; i < numArea + 1; i++)
		{
			pointsPos.at(i).x = 0;
			pointsPos.at(i).y = 0;
		}

		// find the sum cooridnates of all points!
		for (int r = 0; r < height; r++)
		{
			unsigned int* labelLinePtr = &labelPtr[r * width];
			uchar* frameColorLinePtr = &frameColorPtr[r * width];
			for (int c = 0; c < width; c++)
			{
				if (labelLinePtr[c] && frameColorLinePtr[c])
				{
					pointsPos.at(labelLinePtr[c]).x += c;
					pointsPos.at(labelLinePtr[c]).y += r;
				}
			}
		}

		// Average the position
		for (int i = 1; i < numArea + 1; i++)
		{
			pointsPos.at(i).x /= countPtr[i];
			pointsPos.at(i).y /= countPtr[i];
		}

		// Give out the result
		for (int i = 1; i < numArea + 1; i++)
		{
			if (pointsPos.at(i).x && pointsPos.at(i).y)
			{
				cout <<"There is a point at ("+ to_string(pointsPos.at(i).x) + "," + to_string(pointsPos.at(i).y)+ ")" << endl;
			}
		}
		delete data;
	}

	return 0;
}
