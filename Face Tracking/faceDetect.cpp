#include "FlyCapture2.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <wiringPi.h>
#include <faceDetect.h>
#include <softPwm.h>
#include <math.h>

using namespace FlyCapture2;
using namespace std;
using namespace cv;

//GPIO SET UP FOR STEPPER MOTOR
// coil_A_1 - wiringPi pin 5 is Header 18
// coil_A_2 - wiringPi pin 0 is Header 11
// coil_B_1 - wiringPi pin 4 is Header 16
// coil_B_2 - wiringPi pin 7 is Header 7

#define	coil_A_1_pin 5 
#define	coil_A_2_pin 0
#define	coil_B_1_pin 4
#define	coil_B_2_pin 7

//Half stepping
const int StepCount = 8;
int GPIO_Seq[StepCount][4] = {{0,1,0,0}, {0,1,0,1}, {0,0,0,1}, {1,0,0,1}, {1,0,0,0}, {1,0,1,0}, {0,0,1,0}, {0,1,1,0}}; 

//Initialize globals
int pwm_values[100]; 
string file_path = 	"/home/pi/Documents/flycapture.2.11.3.425_armhf/src/imShow/tickCount.txt";
const double SCALE = 4; 
const int PWM_DUTYCYCLE_MAX = 100; 
int w1_prev = 0, w2_prev = 0, w3_prev = 0, w4_prev = 0; 
int centerPosPrev = 0;
int prevStep = 0;
int tickCount = 0;
int speedFactor = 0;
 
int centerPos = 0;

CascadeClassifier face_cascade, profile_cascade; 

int detectFaces(Mat frame) {
	//Convert captured image to gray scale and equalize histogram
	vector<Rect> faces;
	
	Mat frame_gray, scaled_image;
	Mat faceROI;
	cvtColor(frame, frame_gray, COLOR_BGR2GRAY);	// Convert to gray scale
	equalizeHist(frame_gray, frame_gray);     	// Equalize histogram
	
	//Resize new image for post-processing
	resize(frame_gray, scaled_image, Size(0,0), 1/SCALE, 1/SCALE);
	
	//Detect faces
	face_cascade.detectMultiScale(scaled_image, faces, 1.1, 3, 0|CASCADE_SCALE_IMAGE, Size(30,30)); 
	
	Point tempCenter;
	
	//Check if there is a detection
	if(faces.size() > 0) { 
		//Draw ellipse on center of face
		Point center((faces[0].x + faces[0].width/2)*SCALE, (faces[0].y + faces[0].height/2)*SCALE);
		ellipse(frame, center, Size((faces[0].width/2)*SCALE, (faces[0].height/2)*SCALE), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0);
		tempCenter = center; 
	}
	//Check if profile is dectected
	else { 	
		vector<Rect> profile;
		
		//Detect face profiles
		profile_cascade.detectMultiScale(scaled_image, profile, 1.1, 2, 0|CASCADE_SCALE_IMAGE, Size(30, 30));
		
		if(profile.size() > 0) {
			//Draw ellipse on center of profile
			Point center((profile[0].x + profile[0].width/2)*SCALE, (profile[0].y + profile[0].height/2)*SCALE); 
			ellipse(frame, center, Size((profile[0].width/2)*SCALE, (profile[0].height/2)*SCALE), 0, 0, 360, Scalar(255, 0, 255), 4, 8, 0);
			
			tempCenter = center;
		}
		else {
			tempCenter.x = centerPosPrev;
		}
	}
	//Display captured frame
	imshow("Image_Capture", frame);
	
	return tempCenter.x;
}

void createMicrostepPulseIncrements() {
		float increment = (M_PI_2/100); 
		int counter = 0;
		for(float i = 0; i < M_PI_2; i += increment) {
			pwm_values[counter] = (int)100*sin(i); 
			++counter;
		}
}

void microStepUp(int coil) {
	for(int k = 0; k <= PWM_DUTYCYCLE_MAX; ++k) {
		softPwmWrite(coil, pwm_values[k]);
		delayMicroseconds(50); 
    }
}

void microStepDown(int coil){
	for(int k = PWM_DUTYCYCLE_MAX; k >= 0; --k) {
		softPwmWrite(coil, pwm_values[k]);
		delayMicroseconds(50); 
	}
}

//Configures step sequence
void setStep(int w1, int w2, int w3, int w4) {
	
	int difference = 0;
	
	//Check Pin 1
	difference = w1 - w1_prev;  
	if(difference == 0) {}
	else if(difference == 1) {
		microStepUp(coil_A_1_pin);
		return;
	}
	else if(difference == -1) {
		microStepDown(coil_A_1_pin);
		return;
	}
		
	//Check Pin 2
	difference = w2 - w2_prev;	 
	if(difference == 0) {}
	else if(difference == 1) {
		microStepUp(coil_A_2_pin);
		return;
	}
	else if(difference == -1) {
		microStepDown(coil_A_2_pin);
		return;
	}
	
	//Check Pin 3
	difference = w3 - w3_prev;
	if(difference == 0) {}
	else if(difference == 1) {
		microStepUp(coil_B_1_pin);
		return;
	}
	else if(difference == -1) {
		microStepDown(coil_B_1_pin);
		return;
	}
	
	//Check Pin 4
	difference = w4 - w4_prev;
	if(difference == 0) {}
	else if(difference == 1) {
		microStepUp(coil_B_2_pin);
		return;
	}
	else if(difference == -1) {
		microStepDown(coil_B_2_pin);
		return;
	}	
}

//Rotate motor
void rotateMotor(int dir, int numTicks, int wait) {
	//Initialize variables
    int j;
    
    //Do nothing if no difference
    if(dir == centerPosPrev){
		return;
	}
    
    //Rotate cw (head frame of reference)
    if(dir - 320 < -40) {
		tickCount+=numTicks; 
		if(prevStep & 1){
			++prevStep; 
		}
		for(int i = 0; i < numTicks; ++i) {
			prevStep %=8;
			for(j = prevStep; j < prevStep+2; ++j) {
				setStep(GPIO_Seq[j][0], GPIO_Seq[j][1], GPIO_Seq[j][2], GPIO_Seq[j][3]);
				w1_prev = GPIO_Seq[j][0];
				w2_prev = GPIO_Seq[j][1];
				w3_prev = GPIO_Seq[j][2];
				w4_prev = GPIO_Seq[j][3]; 
				delay(wait);	
			}	
			prevStep = j;	
		}
    }
    
    //Rotate ccw (head frame of reference)
    else if(dir - 320 > 40) {
		tickCount-=numTicks;
		if(prevStep == 0) {
			prevStep = 7; 
		}
		if(!(prevStep & 1)){
			--prevStep;
		}
		for(int i = 0; i < numTicks; ++i) {
			for(j = prevStep; j > (prevStep-2); --j) {
				setStep(GPIO_Seq[j][0], GPIO_Seq[j][1], GPIO_Seq[j][2], GPIO_Seq[j][3]);
				w1_prev = GPIO_Seq[j][0];
				w2_prev = GPIO_Seq[j][1];
				w3_prev = GPIO_Seq[j][2];
				w4_prev = GPIO_Seq[j][3]; 
				delay(wait);
			}
			if(j<=-1) {
				prevStep = 7;
			}
			else {
				prevStep = j;
			}
		}
    }
}

void setSpeedFactor(void) {
	speedFactor = (centerPos - centerPosPrev)/10;
	if(speedFactor < 0) {
		speedFactor *= -1;
	}
	else if(speedFactor == 0) {
		speedFactor = 5;
	}
	
	if(speedFactor > 10) {
		speedFactor = 10; 
	}	
}

int main() {	
	//Initialize camera
    Error error;
    Camera camera;
    CameraInfo camInfo;
        
    //GPIO SETUP
    wiringPiSetup();
    
    createMicrostepPulseIncrements(); //Call this to populate pwm_values with sinusoid increments
    
    softPwmCreate(coil_A_1_pin, 0, PWM_DUTYCYCLE_MAX);
    softPwmCreate(coil_A_2_pin, 0, PWM_DUTYCYCLE_MAX);
    softPwmCreate(coil_B_1_pin, 0, PWM_DUTYCYCLE_MAX);
    softPwmCreate(coil_B_2_pin, 0, PWM_DUTYCYCLE_MAX);
    
    //Initialize tickCount file read
    ifstream inFile (file_path);
     
    if (inFile.is_open()) {
			inFile >> tickCount;
			inFile.close();
			if(tickCount != 0) {
				int direction = (tickCount > 0) ? 390 : 250;
				if (tickCount < 0) {
					tickCount *= -1;
				}
				rotateMotor(direction, tickCount, 10); 
				tickCount = 0;
			}
	}
	else {
		cout << "Unable to read in from file" << endl; 
		return false; 
	} 
		  
    //Connect the camera
    error = camera.Connect(0);
    if(error != PGRERROR_OK) {
        cout << "Failed to connect to camera" << endl;     
        return false;
    }

    //Get the camera info and print it out
    error = camera.GetCameraInfo(&camInfo);
    if(error != PGRERROR_OK) {
        cout << "Failed to get camera info from camera" << endl;     
        return false;
    }
    
    cout << camInfo.vendorName << " "
              << camInfo.modelName << " " 
              << camInfo.serialNumber << endl;
              
    //Initialize variables
    Mat frame, image;
    Image rawImage;
    char key = 0;
    int count = 0;

    //Load classifiers from "opencv/data/haarcascades" directory 
    if(!face_cascade.load("/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml")) {
		cout << "Unable to open frontalface classifier" << endl;
		return false;
	}
    if(!profile_cascade.load( "/usr/share/opencv/haarcascades/haarcascade_profileface.xml")) {
		cout << "Unable to open profile classifier" << endl;
		return false;
	}
	
	//Start camera capture
    error = camera.StartCapture();
    if(error == PGRERROR_ISOCH_BANDWIDTH_EXCEEDED) {
        cout << "Bandwidth exceeded" << endl;     
        return false;
    }
    else if(error != PGRERROR_OK) {
        cout << "Failed to start image capture" << endl;     
        return false;
    } 

    //Capture loop
    while(key != 'q') {
        //Get the image
        Error error = camera.RetrieveBuffer(&rawImage);
        if(error != PGRERROR_OK) {
            cout << "capture error" << endl;
			continue;
        }

        //Convert to rgb
        Image rgbImage;
        rawImage.Convert(PIXEL_FORMAT_BGR, &rgbImage);

        //Convert to OpenCV Mat
        unsigned int rowBytes = (double)rgbImage.GetReceivedDataSize()/(double)rgbImage.GetRows();       
        Mat image = Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(),rowBytes);
        count++;
        
        //Check for face every 3 frames
        if(count >= 3) {
			centerPosPrev = centerPos;
	        centerPos = detectFaces(image);
	    	count = 0;
        }
        else { 
        	imshow("Image_Capture", image);
        }
    	
    	setSpeedFactor(); 
		rotateMotor(centerPos, 1, 10/speedFactor);
                
        key = waitKey(5);     
    }

	//Stop capture
    error = camera.StopCapture();
    if(error != PGRERROR_OK) {
        // This may fail when the camera was removed, so don't show 
        // an error message
    }  
    
    //Initialize tickCount file output
	ofstream outFile(file_path, fstream::trunc);
    
	//Write tickCount to file
	if (outFile.is_open()){
		outFile << tickCount;
		outFile.close(); 
	}
	else{
		cout << "Unable to output to file"; 
	}

	//Disconnect camera
    camera.Disconnect();
	
    return 0;
}
