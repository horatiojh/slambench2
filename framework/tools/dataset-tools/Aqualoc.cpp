/*

 Inspired by EUROCMAV.cpp 

 */


#include "./include/Aqualoc.h"

#include <io/SLAMFile.h>
#include <io/sensor/Sensor.h>
#include <io/sensor/CameraSensor.h>
#include <io/sensor/DepthSensor.h>
#include <io/sensor/IMUSensor.h>
#include <io/sensor/GroundTruthSensor.h>
#include <io/SLAMFrame.h>

#include <cstdio>
#include <dirent.h>
#include <cstring>
#include <cmath>

#include <vector>
#include <string>
#include <fstream>

#include <iostream>
#include <regex>

#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>
#include <boost/algorithm/string.hpp>


#include "io/sensor/SensorCollection.h"
#include "io/sensor/CameraSensor.h"
#include "io/SLAMFile.h"
#include "io/SLAMFrame.h"

using namespace slambench::io ;

float FixFileInput (std::string input) {
    boost::smatch match;
    if (boost::regex_match(input,match,boost::regex("^([-0-9.]+)e-0([0-9]+)$"))) {

        float value = std::stof(match[1]);
        float divisor = std::stof(match[2]);
        return (value/(pow(10, divisor)));
    } else {
        return std::stof(input);
    }
}

std::string GetFileNumber (std::string input) {
    //datasets/Aqualoc/sequence_1_no_bag
    boost::smatch match;
    if (boost::regex_match(input,match,boost::regex("^.*_sequence_([0-9]+)_raw_data$"))) {
        // std::cerr << match[1] << std::endl;
        return match[1];
    }
    return "0";
}

std::string GenerateFrameName (std::string input) {

    int len = (int)input.length();

    std::string zeroes = "";

    if (len < 6) {
        // find the number of zeroes to append to the front 
        for (int i = 0; i < (6-len); i++) {
            zeroes.append("0");
        }
    }
    
    std::string editedNum = zeroes + input;
    std::string output = "frame" + editedNum + ".png";
    // std::cerr << output << std::endl;
    return output;
}

SLAMFile* AqualocReader::GenerateSLAMFile () {

    // test print
    std::cerr << " == Generating Aqualoc File ==" << std::endl;

    // begin by creating a new SLAM file
    slambench::io::SLAMFile *slamfile = new slambench::io::SLAMFile();

	
    std::string dataFolder = this->aqualocDatasetFolder;
    std::string fileNumber = GetFileNumber(dataFolder);
    std::string dataType = this->aqualocType;
    std::string rootFolder;
    std::string camCalibFileName;
    YAML::Node cam_calib;
    std::string gtDirFolder;

    rootFolder = this->aqualocInputDir + "/" + dataType;
    // std::cerr << rootFolder << std::endl;
    camCalibFileName = rootFolder + "/" + dataType + "_calibration_files/" + dataType + "_camera_calib.yaml";
    // std::cerr << camCalibFileName << std::endl;
    cam_calib = YAML::LoadFile(camCalibFileName.c_str());
    gtDirFolder = rootFolder + "/" + dataType + "_groundtruth_files";
    


    // initialise the required sensors

    // Grey sensor 
    slambench::io::CameraSensor *greySensor = new slambench::io::CameraSensor("Grey");

    //greySensor->Rate = 
    greySensor->Description = cam_calib["cam0"]["camera_model"].as<std::string>();
    //greySensor->FrameFormat = 
    greySensor->PixelFormat = slambench::io::pixelformat::G_I_8;
    
    //  resolution: [640, 512]
    greySensor->Width = cam_calib["cam0"]["resolution"][0].as<int>();
    greySensor->Height = cam_calib["cam0"]["resolution"][1].as<int>();

    //intrinsics: [413.32595366566017, 413.70198739483686, 305.9507483284928, 259.4439948946375]
    greySensor->Intrinsics[0] = cam_calib["cam0"]["intrinsics"][0].as<float>() / greySensor->Width;
    greySensor->Intrinsics[1] = cam_calib["cam0"]["intrinsics"][1].as<float>() / greySensor->Height;
    greySensor->Intrinsics[2] = cam_calib["cam0"]["intrinsics"][2].as<float>() / greySensor->Width;
    greySensor->Intrinsics[3] = cam_calib["cam0"]["intrinsics"][3].as<float>() / greySensor->Height;

    // now we check if the input file is distorted based on user input, then determine 
    if (this->dist) {
        std::cerr << " Distortion Type: Equidistant " << std::endl;
        greySensor->DistortionType = slambench::io::CameraSensor::distortion_type_t::Equidistant;

        greySensor->EquidistantDistortion[0] = cam_calib["cam0"]["distortion_coeffs"][0].as<float>();
        greySensor->EquidistantDistortion[1] = cam_calib["cam0"]["distortion_coeffs"][1].as<float>();
        greySensor->EquidistantDistortion[2] = cam_calib["cam0"]["distortion_coeffs"][2].as<float>();
        greySensor->EquidistantDistortion[3] = cam_calib["cam0"]["distortion_coeffs"][3].as<float>();
        greySensor->EquidistantDistortion[4] = 0;



    } else if (!this->dist){
        std::cerr << " Distortion Type: None " << std::endl;
        greySensor->DistortionType = slambench::io::CameraSensor::distortion_type_t::NoDistortion;
    } else {
        std::cerr << "Unsupported distortion type for Aqualoc." << std::endl;
        exit(1);
    }
    greySensor->Index = slamfile->Sensors.size();
    slamfile->Sensors.AddSensor(greySensor);

    // gotta match timestamp to frame number using aqua_img.csv
    // store the values for the timestamp and image name in a map, where timestamp is in NS
    // create the map

    std::map <std::string, std::string> timestampMap;
    // std::cerr << " Reading Timestamp File " << std::endl;
   {
    std::string line;
	boost::smatch match;
    std::string numberType = fileNumber;
    std::string inFileName = (dataType == "harbor") ? (dataFolder + "/" + "harbor_img_sequence_" + numberType + ".csv") : (dataFolder + "/" + "img_sequence_" + numberType + ".csv"); 
    std::ifstream infile(inFileName);
    while (std::getline(infile, line)){

        if (line.size() == 0) {
            continue;
        } else if (boost::regex_match(line,match,boost::regex("^\\s*#.*$"))) {
            continue;
        } else if (boost::regex_match(line,match,boost::regex("^([0-9]+),(.*+)$"))) {
            // std::cerr << "Reading line from Timestamp CSV File:" << line << std::endl;
            std::string timestamp = match[1];
            std::string frameID = match[2];
            // std::cerr << "the timestamp value is " + timestamp << std::endl;
            // std::cerr << "the frameID value is " + frameID << std::endl;
            timestampMap.insert(std::pair<std::string, std::string>(frameID, timestamp));
            // std::cerr << "Inserted values into map" << std::endl;

        } else {
            std::cerr << "Unknown line:" << line << std::endl;
        }

    }

	
   }

//     // checking values in map
//    std::for_each(timestampMap.begin(), timestampMap.end(),
// 			[](std::pair<std::string, std::string> element) {
// 				// Accessing KEY from element
// 				std::string word = element.first;
// 				// Accessing VALUE from element.
// 				std::string count = element.second;
// 				std::cout<<word<<" :: "<<count<<std::endl;
// 			});
 

    // std::cerr << " Successfully Read File for Timestamps " << std::endl;


    // we then load the frames for the RGB and Grey sensors from either the dist or undist,
    // based on what the user has indicated
    std::string imgFolder;
    if (dataType == "harbor") {
        imgFolder = dataFolder + "/harbor_images_sequence_" + fileNumber;
    } else {
        imgFolder = dataFolder + "/images_sequence_" + fileNumber;
    }
    // std::cerr << " The IMG Folder used is " + imgFolder << std::endl;
    // DIR *dir = opendir((imgFolder).c_str());
    // dirent *pdir;

    // iterate through the map 
    std::map<std::string, std::string>::iterator it = timestampMap.begin();

    while(it != timestampMap.end()) {  

        
        // Add the original Grey Image

        // std::cerr << " Adding Grey Frame " << std::endl;

        slambench::io::ImageFileFrame *greyFrame = new slambench::io::ImageFileFrame();
        greyFrame->FrameSensor = greySensor;
        greyFrame->Filename = imgFolder + "/" + it->first;
        std::string imgTimestamp = it->second;
        uint64_t timestamp = strtol(imgTimestamp.c_str(), nullptr, 10);
		int timestampS  = timestamp / 1000000000;
		int timestampNS = timestamp % 1000000000;

        greyFrame->Timestamp.S  = timestampS;
		greyFrame->Timestamp.Ns = timestampNS;

        slamfile->AddFrame(greyFrame);

        it++;
    
    }



    slambench::io::IMUSensor *imuSensor = new slambench::io::IMUSensor(dataFolder + "IMUSensor");

    std::string line;

	boost::smatch match;
    // std::cerr << " Reading IMU Values from filename: " + dataFolder + "/" + "aqua_imu.csv" << std::endl;
   
    std::string inFileName = (dataType == "harbor") ? (dataFolder + "/harbor_imu_sequence_" + fileNumber + ".csv") : (dataFolder + "/imu_sequence_" + fileNumber + ".csv");
    std::ifstream infile(inFileName);

    while (std::getline(infile, line)){

        if (line.size() == 0) {
            continue;
        } else if (boost::regex_match(line,match,boost::regex("^\\s*#.*$"))) {
            continue;
        } else if (boost::regex_match(line,match,boost::regex("^([0-9]+),([-0-9.]+),([-0-9.]+),([-0-9.]+),([-0-9.]+),([-0-9.]+),([-0-9.]+)$"))) {

            // std::cerr << "Reading line:" << line << std::endl;
            
            uint64_t timestamp = strtol(std::string(match[1]).c_str(), nullptr, 10);
            int timestampS  = timestamp / 1000000000;
            int timestampNS = timestamp % 1000000000;

            float wx = std::stof(match[2]) ;  
            float wy = std::stof(match[3]) ;  
            float wz = std::stof(match[4]) ;  

            float ax =  std::stof(match[5]); 
            float ay =  std::stof(match[6]); 
            float az =  std::stof(match[7]); 

            slambench::io::SLAMInMemoryFrame *IMUFrame = new slambench::io::SLAMInMemoryFrame();
            IMUFrame->FrameSensor = imuSensor;
            IMUFrame->Timestamp.S  = timestampS;
            IMUFrame->Timestamp.Ns = timestampNS;
            IMUFrame->Data = malloc(imuSensor->GetFrameSize(IMUFrame));

            ((float*)IMUFrame->Data)[0] = wx;
            ((float*)IMUFrame->Data)[1] = wy;
            ((float*)IMUFrame->Data)[2] = wz;

            ((float*)IMUFrame->Data)[3] = ax;
            ((float*)IMUFrame->Data)[4] = ay;
            ((float*)IMUFrame->Data)[5] = az;

            slamfile->AddFrame(IMUFrame);

        } else if (boost::regex_match(line,match,boost::regex("^([0-9]+),([-0-9.e+]+),([-0-9.e+]+),([-0-9.e+]+),([-0-9.e+]+),([-0-9.e+]+),([-0-9.e+]+)$"))) {

            // std::cerr << "Caught the regex here: " << line << std::endl;

            uint64_t timestamp = strtol(std::string(match[1]).c_str(), nullptr, 10);
            int timestampS  = timestamp / 1000000000;
            int timestampNS = timestamp % 1000000000;

            float wx = FixFileInput(match[2]) ;  
            float wy = FixFileInput(match[3]) ;  
            float wz = FixFileInput(match[4]) ;  

            float ax =  FixFileInput(match[5]); 
            float ay =  FixFileInput(match[6]); 
            float az =  FixFileInput(match[7]); 
            
            // std::cerr << "Replaced it with: " << wx << " " << wy << " " << wz << " " << ax << " " << ay << " " << az << std::endl;


            slambench::io::SLAMInMemoryFrame *IMUFrame = new slambench::io::SLAMInMemoryFrame();
            IMUFrame->FrameSensor = imuSensor;
            IMUFrame->Timestamp.S  = timestampS;
            IMUFrame->Timestamp.Ns = timestampNS;
            IMUFrame->Data = malloc(imuSensor->GetFrameSize(IMUFrame));

            ((float*)IMUFrame->Data)[0] = wx;
            ((float*)IMUFrame->Data)[1] = wy;
            ((float*)IMUFrame->Data)[2] = wz;

            ((float*)IMUFrame->Data)[3] = ax;
            ((float*)IMUFrame->Data)[4] = ay;
            ((float*)IMUFrame->Data)[5] = az;

            slamfile->AddFrame(IMUFrame);



        } else {
            std::cerr << "Unknown line:" << line << std::endl;
        }

    }
	
	imuSensor->Index = slamfile->Sensors.size();
    slamfile->Sensors.AddSensor(imuSensor);


    // std::cerr << "Successfully read IMU Sensor Values" << std::endl;

	{
    // Ground truth trajectories (format: frame_id tx ty tz qx qy qz qw)
    // search for the appropriate text file and read it line by line 
    slambench::io::GroundTruthSensor *gtSensor = new slambench::io::GroundTruthSensor(gtDirFolder + "GTSensor");
    std::string line;
	boost::smatch match;

    std::string seqNumber = boost::regex_replace(
        dataFolder,
        boost::regex("[^0-9]*([0-9]+).*"),
        std::string("\\1")
        );
    std::string fileName;
    if (dataType == "harbor") {
        fileName = rootFolder + "/harbor_groundtruth_files/harbor_colmap_traj_sequence_" + fileNumber + ".txt"; 
    } else {
        fileName = rootFolder + "/archaeo_groundtruth_files/colmap_traj_sequence_" + fileNumber + ".txt"; 
    }

	std::ifstream inFile(fileName);

    if (!inFile) {
        std::cerr << "Unable to open file " + fileName;
        exit(1);  
    }

    // std::cerr << " Successfully located file " + fileName << std::endl;

	while (std::getline(inFile, line)){

		if (boost::regex_match(line,match,boost::regex("^([0-9]+).0 ([-]?[0-9.]+) ([-]?[0-9.]+) ([-]?[0-9.]+) ([-]?[0-9.]+) ([-]?[0-9.]+) ([-]?[0-9.]+) ([-]?[0-9.]+)$"))) {

            // The first parameter is the frame number, so we have to find the timestamp via this information 
            
            std::string frameNumber = std::string(match[1]);
            std::string modifiedFrame = GenerateFrameName(frameNumber);
            std::string time = timestampMap.find(modifiedFrame)->second; 
            uint64_t timestamp = strtol(time.c_str(), nullptr, 10);
            
            int timestampS  = timestamp / 1000000000;
			int timestampNS = timestamp % 1000000000;

			float tx = std::stof(match[2]); 
			float ty = std::stof(match[3]); 
			float tz = std::stof(match[4]); 

			float qx =  std::stof(match[5]);  
			float qy =  std::stof(match[6]);  
			float qz =  std::stof(match[7]);  
            float qw =  std::stof(match[8]);  

            // QUESTION: I transformed the data in the same way as EUROCMAV, is this correct?
			Eigen::Matrix3f rotationMat = Eigen::Quaternionf(qw,qx,qy,qz).toRotationMatrix();
			Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
			pose.block(0,0,3,3) = rotationMat;
			pose.block(0,3,3,1) << tx , ty , tz;


			slambench::io::SLAMInMemoryFrame *gtFrame = new slambench::io::SLAMInMemoryFrame();
			gtFrame->FrameSensor = gtSensor;
			gtFrame->Data = malloc(gtSensor->GetFrameSize(gtFrame));
            gtFrame->Timestamp.S = timestampS;
			gtFrame->Timestamp.Ns = timestampNS;

			memcpy(gtFrame->Data,pose.data(),gtSensor->GetFrameSize(gtFrame));

			slamfile->AddFrame(gtFrame);

            // std::cerr << "Frame successfully added" << std::endl;

		} else if (boost::regex_match(line,match,boost::regex("^([0-9]+).0 ([-]?[0-9.e-]+) ([-]?[0-9.e-]+) ([-]?[0-9.e-]+) ([-]?[0-9.e-]+) ([-]?[0-9.e-]+) ([-]?[0-9.e-]+) ([-]?[0-9.e-]+)$"))){
            
            std::string frameNumber = std::string(match[1]);
            // std::cerr << "Frame Number is " + frameNumber << std::endl;
            std::string modifiedFrame = GenerateFrameName(frameNumber);
            std::string time = timestampMap.find(modifiedFrame)->second; 
            uint64_t timestamp = strtol(time.c_str(), nullptr, 10);

            int timestampS  = timestamp / 1000000000;
			int timestampNS = timestamp % 1000000000;

            float tx = FixFileInput(match[2]); 
			float ty = FixFileInput(match[3]); 
			float tz = FixFileInput(match[4]); 

			float qx =  FixFileInput(match[5]);  
			float qy =  FixFileInput(match[6]);  
			float qz =  FixFileInput(match[7]);  
            float qw =  FixFileInput(match[8]);  

            // QUESTION: I transformed the data in the same way as EUROCMAV, is this correct?
			Eigen::Matrix3f rotationMat = Eigen::Quaternionf(qw,qx,qy,qz).toRotationMatrix();
			Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
			pose.block(0,0,3,3) = rotationMat;
			pose.block(0,3,3,1) << tx , ty , tz;


			slambench::io::SLAMInMemoryFrame *gtFrame = new slambench::io::SLAMInMemoryFrame();
			gtFrame->FrameSensor = gtSensor;
			gtFrame->Data = malloc(gtSensor->GetFrameSize(gtFrame));
            gtFrame->Timestamp.S = timestampS;
			gtFrame->Timestamp.Ns = timestampNS;

			memcpy(gtFrame->Data,pose.data(),gtSensor->GetFrameSize(gtFrame));

			slamfile->AddFrame(gtFrame);

        } else {
			std::cerr << "Unknown line:" << line << std::endl;
		}


	}

    gtSensor->Index = slamfile->Sensors.size();
    slamfile->Sensors.AddSensor(gtSensor);
	}

    std::cerr << " Successfully created SLAMFile " << std::endl;
	
	return slamfile;

}





