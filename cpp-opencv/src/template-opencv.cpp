/*
 * Copyright (C) 2020  Christian Berger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// Include the single-file, header-only middleware libcluon to create high-performance microservices
#include "cluon-complete.hpp"
// Include the OpenDLV Standard Message Set that contains messages that are usually exchanged for automotive or robotic applications 
#include "opendlv-standard-message-set.hpp"

// Include the GUI and image processing header files from OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <bits/stdc++.h>
#include <math.h>
#include <iostream> 

using namespace cv; 
using namespace std; 

bool isYellowLeft = false;      // Boolean value for if we have yellow cones on our left
bool isClockwiseKnown = false;  // Boolean value for if we know what direction we're going 
double correctFrames;       // Number of correct frames (by the end of each recording)
double frames;              // Total numver of rames (by the end of each recording)

// Number of frames for each case
float case_1 = 0;
float case_2 = 0;
float case_3 = 0;
float case_4 = 0;
float case_5 = 0;
float case_6 = 0;
//Number of correct calculations for each frame
float c_1 = 0;
float c_2 = 0;
float c_3 = 0;
float c_4 = 0;
float c_5 = 0;
float c_6 = 0;

// High and low global variable values for blue and yellow colors
cv::Scalar blueLow = cv::Scalar(100, 100, 40);
cv::Scalar blueHigh = cv::Scalar(133, 255, 255);
cv::Scalar yellowLow = cv::Scalar(15, 50, 130);
cv::Scalar yellowHigh = cv::Scalar(25, 185, 255);

//------------------ Function declaration -------------------
double calculateAverageAccuracy();
void testPerformance(float groundSteering, float calculatedAngle);
std::array<cv::Point2f,2> drawContourWithCentroidPoint(cv::Mat inputImage,cv::Mat outputImage, int contourArea, cv::Scalar centroidColor);
float calculateAngle(std::array<cv::Point2f,2> blueCones, std::array<cv::Point2f,2> yellowCones, float steeringRequest);
bool testPerformanceV2(float groundSteering, float calculatedAngle);
//------------------ Function declaration -------------------

int32_t main(int32_t argc, char **argv) {
    int32_t retCode{1};
    // Parse the command line parameters as we require the user to specify some mandatory information on startup.
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if ( (0 == commandlineArguments.count("cid")) ||
         (0 == commandlineArguments.count("name")) ||
         (0 == commandlineArguments.count("width")) ||
         (0 == commandlineArguments.count("height")) ) {
        std::cerr << argv[0] << " attaches to a shared memory area containing an ARGB image." << std::endl;
        std::cerr << "Usage:   " << argv[0] << " --cid=<OD4 session> --name=<name of shared memory area> [--verbose]" << std::endl;
        std::cerr << "         --cid:    CID of the OD4Session to send and receive messages" << std::endl;
        std::cerr << "         --name:   name of the shared memory area to attach" << std::endl;
        std::cerr << "         --width:  width of the frame" << std::endl;
        std::cerr << "         --height: height of the frame" << std::endl;
        std::cerr << "Example: " << argv[0] << " --cid=253 --name=img --width=640 --height=480 --verbose" << std::endl;
    }
    else {
        // Extract the values from the command line parameters
        const std::string NAME{commandlineArguments["name"]};
        const uint32_t WIDTH{static_cast<uint32_t>(std::stoi(commandlineArguments["width"]))};
        const uint32_t HEIGHT{static_cast<uint32_t>(std::stoi(commandlineArguments["height"]))};
        const bool VERBOSE{commandlineArguments.count("verbose") != 0};

        // Attach to the shared memory.
        std::unique_ptr<cluon::SharedMemory> sharedMemory{new cluon::SharedMemory{NAME}};
        if (sharedMemory && sharedMemory->valid()) {
            std::clog << argv[0] << ": Attached to shared memory '" << sharedMemory->name() << " (" << sharedMemory->size() << " bytes)." << std::endl;

            // Interface to a running OpenDaVINCI session where network messages are exchanged.
            // The instance od4 allows you to send and receive messages.
            cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

            opendlv::proxy::GroundSteeringRequest gsr;
            std::mutex gsrMutex;
            auto onGroundSteeringRequest = [&gsr, &gsrMutex](cluon::data::Envelope &&env){
                // The envelope data structure provide further details, such as sampleTimePoint as shown in this test case:
                // https://github.com/chrberger/libcluon/blob/master/libcluon/testsuites/TestEnvelopeConverter.cpp#L31-L40
                std::lock_guard<std::mutex> lck(gsrMutex);
                gsr = cluon::extractMessage<opendlv::proxy::GroundSteeringRequest>(std::move(env));
                //std::cout << "lambda: groundSteering = " << gsr.groundSteering() << std::endl;
            };

            od4.dataTrigger(opendlv::proxy::GroundSteeringRequest::ID(), onGroundSteeringRequest);

            // Endless loop; end the program by pressing Ctrl-C.
            while (od4.isRunning()) {
                 // OpenCV data structure to hold an image.
                cv::Mat img;

                // Wait for a notification of a new frame.
                sharedMemory->wait();

                // Lock the shared memory.
                sharedMemory->lock();
                {
                    // Copy the pixels from the shared memory into our own data structure.
                    cv::Mat wrapped(HEIGHT, WIDTH, CV_8UC4, sharedMemory->data());
                    img = wrapped.clone();
                }
                // TODO: Here, you can add some code to check the sampleTimePoint when the current frame was captured.
                auto [_, tstamp] = sharedMemory->getTimeStamp();
                auto ms = static_cast<int64_t>(tstamp.seconds()) * static_cast<int64_t>(1000 * 1000) + static_cast<int64_t>(tstamp.microseconds());
                // Crop some of the dead space
                img = img(cv::Rect(0, 265, 640, 140));
                sharedMemory->unlock();

                cluon::data::TimeStamp ts = cluon::time::now();
                uint32_t seconds = ts.seconds();
                std::stringstream stream;
                std::time_t time = static_cast<time_t>(seconds);
                tm *p_time = gmtime(&time);
                stream << p_time->tm_year + 1900; // needed to add 1900 because ctime tm_year is current year - 1900

                if (p_time->tm_mon < 10)
                {
                    stream << "0";
                }
                stream << p_time->tm_mon + 1 // based on 0-11 range, +1 to correct
                       << "-";
                if (p_time->tm_mday < 10)
                {
                    stream << "0";
                }
                stream << p_time->tm_mday
                       << "T";
                if (p_time->tm_hour < 10)
                {
                    stream << "0";
                }
                stream << p_time->tm_hour + 2 // same as with the month, 0-23 hour range
                       << ":";
                if (p_time->tm_min < 10)
                {
                    stream << "0";
                }
                stream << p_time->tm_min 
                       << ":";
                if (p_time->tm_sec < 10)
                {
                    stream << "0";
                }
                stream << p_time->tm_sec
                       << "Z";
                std::string date = stream.str();

                std::string output = "Now: " + date + "; ts: " + std::to_string(ms) + ";";

                //--------------- Color detection section ---------------
                // Create a new mat image
                cv::Mat imgHSV;
                // Copy the original image to the new one
                img.copyTo(imgHSV);
                // Convert the new image to the hsv color space
                cv::cvtColor(imgHSV, imgHSV, cv::COLOR_BGR2HSV);

                cv::Mat imgColorSpaceBLUE;
                cv::Mat imgColorSpaceYELLOW;

                // THIS DETECTS BLUE CONES
                cv::inRange(imgHSV, blueLow, blueHigh, imgColorSpaceBLUE);
                // THIS DETECTS YELLOW CONES
                cv::inRange(imgHSV, yellowLow, yellowHigh, imgColorSpaceYELLOW);
                // combines the two resulted images
                cv::Mat imgColorSpace = imgColorSpaceBLUE | imgColorSpaceYELLOW;
                //--------------- Color detection section ---------------

                //---------------- Noise removal ------------------------
                // fill holes in objects - blue cones
                cv::dilate(imgColorSpaceBLUE, imgColorSpaceBLUE, getStructuringElement(MORPH_ELLIPSE, Size(8, 8))); 
                cv::erode(imgColorSpaceBLUE, imgColorSpaceBLUE, getStructuringElement(MORPH_ELLIPSE, Size(8, 8)));
                // remove small objects - blue cones
                cv::erode(imgColorSpaceBLUE, imgColorSpaceBLUE, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
                cv::dilate(imgColorSpaceBLUE, imgColorSpaceBLUE, getStructuringElement(MORPH_ELLIPSE, Size(7, 7)) ); 
                
                // fill holes in objects - yellow cones
                cv::dilate(imgColorSpaceYELLOW, imgColorSpaceYELLOW, getStructuringElement(MORPH_ELLIPSE, Size(8, 8))); 
                cv::erode(imgColorSpaceYELLOW, imgColorSpaceYELLOW, getStructuringElement(MORPH_ELLIPSE, Size(8, 8)));
                // remove small objects - yellow cones
                cv::erode(imgColorSpaceYELLOW, imgColorSpaceYELLOW, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
                cv::dilate(imgColorSpaceYELLOW, imgColorSpaceYELLOW, getStructuringElement(MORPH_ELLIPSE, Size(7, 7)) ); 
                //---------------- Noise removal ------------------------

                // Arrays for the deteced blue and yellow cones
                std::array<cv::Point2f,2> blueCones;
                std::array<cv::Point2f,2> yellowCones;
                
                // Scalar for the red color to be given to the detectection method 
                cv::Scalar red = cv::Scalar(0,0,255);
                // Calling cone detecting methods
                blueCones = drawContourWithCentroidPoint(imgColorSpaceBLUE, img, 20, red);
                yellowCones = drawContourWithCentroidPoint(imgColorSpaceYELLOW, img, 40, red);
                
                // Getting the ground steering angle for testing purposes
                float groundSteering = gsr.groundSteering();
                // Calling the angle calculator
                float calculatedAngle = calculateAngle(blueCones, yellowCones, groundSteering);
                // Counting the number of frames
                frames++;
                // Testing the overall performance (for this frame)
                testPerformance(groundSteering, calculatedAngle);

                //std::cout << "His: = " << groundSteering << std::endl << "Ours: " << calculatedAngle << std::endl;

                std::cout << "group_08;" << std::to_string(ms) << ";" << calculatedAngle << std::endl;
                
                // If you want to access the latest received ground steering, don't forget to lock the mutex:
                {
                    std::lock_guard<std::mutex> lck(gsrMutex);
                    //std::cout << "main: groundSteering = " << gsr.groundSteering() << std::endl;
                }

                // Display image on your screen.
                if (VERBOSE) {
                    cv::putText(img,                        // target image
                            output,                     // text
                            cv::Point(0, img.rows / 8), // top-left position
                            cv::FONT_HERSHEY_PLAIN,
                            1.4,
                            CV_RGB(255, 255, 255),          // font color
                            1);
                    cv::imshow("Black & white Image", imgColorSpace); 
                    cv::imshow(sharedMemory->name().c_str(), img);
                    cv::waitKey(1);
                }
            }
            // Calculating the average accuracy
            double result = calculateAverageAccuracy();
            std::cout << "Looking for the accuracy? Average Accuracy: " << result << std::endl;

            // Calculaating the accuracy for each case
            float a_1 = (c_1 / case_1) * 100;
            float a_2 = (c_2 / case_2) * 100;
            float a_3 = (c_3 / case_3) * 100;
            float a_4 = (c_4 / case_4) * 100;
            float a_5 = (c_5 / case_5) * 100;
            float a_6 = (c_6 / case_6) * 100;

            // Printing the result
            std::cout << "Case 1: " << case_1 << "-" << a_1 << std::endl 
            << "Case 2: " << case_2 << "-" << a_2 << std::endl 
            << "Case 3: " << case_3 << "-" << a_3 << std::endl
            << "Case 4: " << case_4 << "-" << a_4 << std::endl 
            << "Case 5: " << case_5 << "-" << a_5 << std::endl 
            << "Case 6: " << case_6 << "-" << a_6 << std::endl;
        }
        retCode = 0;
    }
    return retCode;
}

//This method does the counturing/shape-detection.
//the color detection happens outside this method and it recieves the black and white resulted image as an input
//this method also recieves the output image that the alterations (dots on the cones) should appear on
//moreover, the parameter "contourArea" is also given to the method to later on be used for deciding if a detected element should 
//be considered a cone or not (based on its size/area)
//the "centroidColor" is just the color that dots should be 
//This method returns the centre point of the cone (X,Y coordinates)
std::array<cv::Point2f,2> drawContourWithCentroidPoint(cv::Mat inputImage, cv::Mat outputImage, int contourArea, cv::Scalar centroidColor)
{
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    // convert to gray scale
    // draw blue cones' contour
    cv::Mat img_channels[3];
    cv::split(inputImage, img_channels);
    cv::Mat img_gray = img_channels[0];
    cv::findContours(img_gray, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    // get the moments
    std::vector<cv::Moments> mu(contours.size());
    std::vector<cv::Point2f> mc(mu.size());
    // Variable for the point of the cone
    cv::Point2f cone;
    // Size of the array to be returned
    const int MAX = 2;
    // Array of the detected cones
    std::array<cv::Point2f,MAX> cones;
    // Initial number of cones
    int num_cones = 1;
    // You might laugh at this but comparing an actual cone against a null one (dummy_cone), was the only way we could see if there exists a cone
    cv::Point2f dummy_cone;
    // Max distance allowed for "detecting" and adding a cone to the array of detected cones
    float distance = 30.0;

    if (contours.size() > 0)        // If there's something
    {
        for (int i = 0; i < contours.size(); i++)       // Going through all the contours
        {
            if (cv::contourArea(contours[i]) > contourArea)     // If it reaches the treshhold (if it's big enough to be considered a cone)
            {
                mu[i] = cv::moments(contours[i], false);        // Get the moment
            }
        }
        // get the centroid of figures.
        for (int i = 0; i < mu.size(); i++)
        {
            mc[i] = cv::Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
        }
        for (int i = 0; i < mu.size(); i++)
        { 
            if(mc[i].x > 0 && num_cones != 3){      // If there exists a cone and the array is not full
                if (num_cones != 2)
                {
                    circle(outputImage, mc[i], 4, centroidColor, -1, 8, 0);     // Draw a ciscle on the object
                    // then we have a value
                    cone = mc[i];
                    cones[i] = cone;
                    num_cones++;
                } else {
                    cone = mc[i];
                    if (((cone.y - cones[0].y) > distance) || ((cone.y - cones[0].y) < -1 * distance) || ((cone.x - cones[0].x) > distance) || ((cone.x - cones[0].x) < -1 * distance))
                    {
                        circle(outputImage, mc[i], 4, centroidColor, -1, 8, 0);
                        cones[i] = cone;
                        num_cones++;
                    }
                }
            }
        }
    }
    return cones;
}

float calculateAngle(std::array<cv::Point2f,2> blueCones, std::array<cv::Point2f,2> yellowCones, float steeringRequest) {

    float calculatedAngle = 0.0;    // Our calculated angle result
    cv::Point2f dummy_cone;         // You might laugh at this but comparing an actual cone against a null one (dummy_cone), was the only way we could see if there exists a cone
    float midd_point = 0;           // The middle X value
    float negative = -1.0;          // Variable for negative number
    float offset = 0;               // The distance from the middle point to the center of the screen
    // Magic values for the calculations of the steering angle
    float c1 = 0.00035;            
    float c2 = 0.18;
    float c3 = 0.0000100;
    float c4 = 0.0000100;
    float width = 640.0;            // Width of the screen
    float midd_width = 320.0;       // Half of the size of the width

	if(blueCones[0] != dummy_cone && yellowCones[0] != dummy_cone) {        // There exists a cone of each color
        //std::cout << "Case 1" << std::endl;
        case_1++;                                                           // Counting the number of frames for case 1
        if (yellowCones[0].x < blueCones[0].x) {                            // Check if the yellow cone is on the left
            isYellowLeft = true;                                            // if yes true
        }
        else {
            isYellowLeft = false;                                           // if not false
        }
        isClockwiseKnown = true;                                            // we know the direction we're going
		midd_point = (blueCones[0].x + yellowCones[0].x)/2;                 // calculate the midd ponit of the two cones (X)
        offset = midd_width - midd_point;                                   // get the offset of that from the center point
        calculatedAngle = offset * c1;                                      // multiply by the constant
        bool fact = testPerformanceV2(steeringRequest, calculatedAngle);    // check if the angle is in the correct range for this case in specific
        if(fact)                                                            // if yes, increase the correct number of frames for this case by one
            c_1++;
	} else if(blueCones[0] != dummy_cone && blueCones[1] != dummy_cone) {       // Only two blue cones are detected
        //std::cout << "Case 2" << std::endl;
        blueCones[0].y = 480 - blueCones[0].y;                              // reverse the height
        blueCones[1].y = 480 - blueCones[1].y;                              // reverse the height
        case_2++;                                                           // Counting the number of frames for case 2
        if (blueCones[0].y != blueCones[1].y) {                             // if the cones' Y values are not the same

            if (blueCones[0].y < 70)                                        // if the distance of the first cone is less than 70, return 0.0
            {
                calculatedAngle = 0.0;
            } else {
            
            calculatedAngle = (blueCones[0].x - blueCones[1].x);            // otherwise, get the distnace of the two cones (X)
            calculatedAngle = calculatedAngle * 0.0005;                     // multiply that by the constant
            }
        } else {
            calculatedAngle = 0.0;                                          // otherwise, return 0.0
        }   
        bool fact = testPerformanceV2(steeringRequest, calculatedAngle);    // check if the angle is in the correct range for this case in specific
        if(fact)                                                            // if yes, increase the correct number of frames for this case by one
            c_2++;  
	} else if(yellowCones[0] != dummy_cone && yellowCones[1] != dummy_cone) {       //Only two yellow cones are detected
        //std::cout << "Case 3" << std::endl;
        // Same as above
        yellowCones[0].y = 480 - yellowCones[0].y;                                  
        yellowCones[1].y = 480 - yellowCones[1].y;                                  
        case_3++;
        if (yellowCones[0].y != yellowCones[1].y){                 
            // The difference here is that we calculate the inverse of the gradient and we get the atan    
            calculatedAngle = negative * ((yellowCones[0].x - yellowCones[1].x) / (yellowCones[0].y - yellowCones[1].y));
            calculatedAngle = atan(calculatedAngle/100);
            calculatedAngle = calculatedAngle * c2;
        } else {
            calculatedAngle = 0.0;
        }
        bool fact = testPerformanceV2(steeringRequest, calculatedAngle);
        if(fact)
            c_3++;
	} else if(blueCones[0] != dummy_cone) {                // Only one blue cone is detected
        //std::cout << "Case 4" << std::endl;                       // For case 4 and 5, we get the distance of the cone (x), from 
        case_4++;                                                   // the coresponding side (left or right) and then multiply that
		if (isClockwiseKnown) {                                     // by a constant. Before that we check if we know the direction
            if (isYellowLeft) {                                     // if yes the first block, if not we try to guess by doing line 430
                calculatedAngle = width - blueCones[0].x;           // The rest is as above
                calculatedAngle = calculatedAngle * c3;
            } else {
                calculatedAngle = blueCones[0].x;
                calculatedAngle = negative * calculatedAngle * c3;
            }
		} else {
            if (blueCones[0].x > midd_width) {
                calculatedAngle = width - blueCones[0].x;
                calculatedAngle = calculatedAngle * c4;
            } else {
                calculatedAngle = blueCones[0].x;
                calculatedAngle = negative * calculatedAngle * c4;
            } 
        }
        bool fact = testPerformanceV2(steeringRequest, calculatedAngle);
        if(fact)
            c_4++;
	} else if(yellowCones[0] != dummy_cone) {           // Only one yellow cone is deteceted
        //std::cout << "Case 5" << std::endl;
        case_5++;
		if (isClockwiseKnown) {                                             // same as above
            if (!isYellowLeft) {
                calculatedAngle = width - yellowCones[0].x;
                calculatedAngle = calculatedAngle * c3;
            } else {
                calculatedAngle = yellowCones[0].x;
                calculatedAngle = negative * calculatedAngle * c3;
            }
		} else {
            if (yellowCones[0].x > midd_width) {
                calculatedAngle = width - yellowCones[0].x;
                calculatedAngle = calculatedAngle * c4;
            } else {
                calculatedAngle = yellowCones[0].x;
                calculatedAngle = negative * calculatedAngle * c4;
            }
            
        }
        bool fact = testPerformanceV2(steeringRequest, calculatedAngle);
        if(fact)
            c_5++;
	} else {                // No cone is detected
        //std::cout << "Case 6" << std::endl;
        case_6++;
        calculatedAngle = 0.0;                                                  // if no cones are detected, just go straight the head and hope for the best
        bool fact = testPerformanceV2(steeringRequest, calculatedAngle);
        if(fact)
            c_6++;
    }
    
    return calculatedAngle;
}

// Method for calculating the average accuracy of the whole thing
void testPerformance(float groundSteering, float calculatedAngle){
    // Based on the criteria provided by the customer, we check if the calculated angle is in the desired range of 70%-130%
    if(groundSteering == 0.0){
        if(calculatedAngle < 0.05 && calculatedAngle > -0.05){
            correctFrames++;
        } 
    }
    else if(groundSteering > 0.0){
        if(groundSteering * 0.7 < calculatedAngle && calculatedAngle < 1.3 * groundSteering){
            correctFrames++;
        } 
    }
    else {
        if(groundSteering * 0.7 > calculatedAngle && groundSteering * 1.3 < calculatedAngle){
            correctFrames++;
        } 
    }
}
// Method for calculating the average accuarcy of each case
bool testPerformanceV2(float groundSteering, float calculatedAngle){
    // Based on the criteria provided by the customer, we check if the calculated angle is in the desired range of 70%-130%
    if(groundSteering == 0.0){
        if(calculatedAngle < 0.05 && calculatedAngle > -0.05){
            return true;
        } else {
            return false;
        }
    }
    else if(groundSteering > 0.0){
        if(groundSteering * 0.7 < calculatedAngle && calculatedAngle < 1.3 * groundSteering){
            return true;
        } else {
            return false;
        }
    }
    else {
        if(groundSteering * 0.7 > calculatedAngle && groundSteering * 1.3 < calculatedAngle){
            return true;
        } else {
            return false;
        }
    }
}
// Self explanatory
double calculateAverageAccuracy(){
    return (correctFrames/frames) * 100;
}
