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


// create only for one sequence, read user input and then create SLAMFile for that sequence 

using namespace slambench::io ;
// work with the generateslam file
// create an IMU sensor, grey sensor, and GT sensor, and mock RGB sensor 

SLAMFile* AqualocReader::GenerateSLAMFile () {

    // test print
    std::cerr << " == Generating Aqualoc File ==" << std::endl;

    // begin by creating a new SLAM file
    slambench::io::SLAMFile *slamfile = new slambench::io::SLAMFile();

	std::string rootFolder = this->aqualocInputDir;
    std::string dataFolder = this->aqualocDatasetFolder;

    // begin by reading the YAML file for the config settings
	std::string camCalibFileName = rootFolder + "/camchain-calib_aqualoc.yaml";
	std::string imuCalibFileName = rootFolder + "/MPU9250_imu_param.yaml";
    YAML::Node cam_calib = YAML::LoadFile(camCalibFileName.c_str());
    YAML::Node imu_calib = YAML::LoadFile(imuCalibFileName.c_str());
    
    // individual folders don't have YAML files, so the format will be a little different from EUROCMAV
    // we read through the csv files in the selected folder/dataset
    // impt to note that the ground truth trajectories are in a separate folder though
    std::string gtDirFolder = rootFolder + "/aqualoc_gt_trajectories";


    // initialise the required sensors

    // Grey sensor 
    slambench::io::CameraSensor *greySensor = new slambench::io::CameraSensor(dataFolder + "GreySensor");

    greySensor->Index = slamfile->Sensors.size();
    //greySensor->Rate = 
    greySensor->Description = cam_calib["cam0"]["camera_model"].as<std::string>();
    //greySensor->FrameFormat = 
    //greySensor->PixelFormat = slambench::io::pixelformat::G_I_8;
    
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

    // here we create an RGB equivalent sensor (with the same settings as the grey sensor)
    
    slambench::io::CameraSensor *rgbSensor = new slambench::io::CameraSensor(dataFolder + "RgbSensor");
    rgbSensor->Index = slamfile->Sensors.size();
    //rgbSensor->Rate = 
    rgbSensor->Description = cam_calib["cam0"]["camera_model"].as<std::string>();
    //rgbSensor->FrameFormat = 
    //rgbSensor->PixelFormat = slambench::io::pixelformat::G_I_8;
    
    rgbSensor->Width = cam_calib["cam0"]["resolution"][0].as<int>();
    rgbSensor->Height = cam_calib["cam0"]["resolution"][1].as<int>();
    rgbSensor->Intrinsics[0] = cam_calib["cam0"]["intrinsics"][0].as<float>() / rgbSensor->Width;
    rgbSensor->Intrinsics[1] = cam_calib["cam0"]["intrinsics"][1].as<float>() / rgbSensor->Height;
    rgbSensor->Intrinsics[2] = cam_calib["cam0"]["intrinsics"][2].as<float>() / rgbSensor->Width;
    rgbSensor->Intrinsics[3] = cam_calib["cam0"]["intrinsics"][3].as<float>() / rgbSensor->Height;


    if (this->dist) {
        rgbSensor->DistortionType = slambench::io::CameraSensor::distortion_type_t::Equidistant;

        rgbSensor->EquidistantDistortion[0] = cam_calib["cam0"]["distortion_coeffs"][0].as<float>();
        rgbSensor->EquidistantDistortion[1] = cam_calib["cam0"]["distortion_coeffs"][1].as<float>();
        rgbSensor->EquidistantDistortion[2] = cam_calib["cam0"]["distortion_coeffs"][2].as<float>();
        rgbSensor->EquidistantDistortion[3] = cam_calib["cam0"]["distortion_coeffs"][3].as<float>();
        rgbSensor->EquidistantDistortion[4] = 0;

    } else if (!this->dist){
        rgbSensor->DistortionType = slambench::io::CameraSensor::distortion_type_t::NoDistortion;
    } else {
        std::cerr << "Unsupported distortion type for Aqualoc." << std::endl;
        exit(1);
    }

    // std::cerr << " Successfully created RGB Sensor " << std::endl;

    // TODO
    // gotta match timestamp to frame number using aqua_img.csv
    // store the values for the timestamp and image name in a map
    // create the map
    std::map <std::string, std::string> timestampMap;
    // std::cerr << " Reading Timestamp File " << std::endl;
   {
    std::string line;
	boost::smatch match;
	std::ifstream infile(dataFolder + "/" + "aqua_img.csv");

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
    std::string imgFolder = this->dist ? (dataFolder + "/undist_images") : (dataFolder + "/dist_images");
    // std::cerr << " The IMG Folder used is " + imgFolder << std::endl;
    // DIR *dir = opendir((imgFolder).c_str());
    // dirent *pdir;

    // iterate through the map 
    std::map<std::string, std::string>::iterator it = timestampMap.begin();

    while(it != timestampMap.end()) {  

        
        // Add the original Grey Image

        slambench::io::ImageFileFrame *greyFrame = new slambench::io::ImageFileFrame();
        greyFrame->FrameSensor = greySensor;
        greyFrame->Filename = imgFolder + it->first;
        std::cerr << " The Filename is " + it->first << std::endl;

        // lookup in the map for timestamp value based on the filename
        // std::string imgFilename = pdir->d_name;


        // TODO: do an if statement to catch if it finds the image filename timestamp
        std::string imgTimestamp = it->second;
        // std::cerr << " The Timestamp value is " + imgTimestamp << std::endl;

        uint64_t timestamp = strtol(imgTimestamp.c_str(), nullptr, 10);
		int timestampS  = timestamp / 1000000000;
		int timestampNS = timestamp % 1000000000;

        greyFrame->Timestamp.S  = timestampS;
		greyFrame->Timestamp.Ns = timestampNS;

        slamfile->AddFrame(greyFrame);

        it++;

        // Add the clone RGB, code is the same as the grey sensor above
        // QUESTION: would it be better to just create the grey sensor then duplicate with something like
        // slambench::io::ImageFileFrame *rgbFrame = greyFrame;
        // slamfile->AddFrame(rgbFrame);
        // since all the info is identical?

        // if (this->rgb) {
            
        //     slambench::io::ImageFileFrame *rgbFrame = new slambench::io::ImageFileFrame();
        //     rgbFrame->FrameSensor = rgbSensor;
        //     rgbFrame->Filename = imgFolder + pdir->d_name;

        //     std::string imgFilename = pdir->d_name;
        //     std::string imgTimestamp = timestampMap.find(imgFilename) -> second;
        //     // std::cerr << " The Timestamp value is " + imgTimestamp << std::endl;

        //     uint64_t timestamp = strtol(imgTimestamp.c_str(), nullptr, 10);
        //     int timestampS  = timestamp / 1000000000;
        //     int timestampNS = timestamp % 1000000000;

        //     rgbFrame->Timestamp.S  = timestampS;
        //     rgbFrame->Timestamp.Ns = timestampNS;

        //     // rgbFrame->Timestamp.Ns = timestamp % 1000000000;
        //     slamfile->AddFrame(rgbFrame);
        // }
    
    }

    slambench::io::IMUSensor *imuSensor = new slambench::io::IMUSensor(dataFolder + "IMUSensor");

    std::string line;

	boost::smatch match;
    // std::cerr << " Reading IMU Values from filename: " + dataFolder + "/" + "aqua_imu.csv" << std::endl;
	std::ifstream infile(dataFolder + "/" + "aqua_imu.csv");

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

		} else {
			
            std::cerr << "Unknown line:" << line << std::endl;
            // QUESTION
            // unknown line mainly happens because of data representation in the csv, where they use scientific notation E-05 etc.
            // if it's an unknown line, does it affect the generation of the SLAM file and do 
            // we have to give it a mock value? If so, we can use the previous line or give it a dummy value
            // but the error still occurs 

			// float wx = 0;
			// float wy = 0;
			// float wz = 0;

			// float ax =  0;
			// float ay =  0;
			// float az =  0;

			// slambench::io::SLAMInMemoryFrame *IMUFrame = new slambench::io::SLAMInMemoryFrame();
			// IMUFrame->FrameSensor = imuSensor;
			// IMUFrame->Data = malloc(imuSensor->GetFrameSize(IMUFrame));

			// ((float*)IMUFrame->Data)[0] = wx;
			// ((float*)IMUFrame->Data)[1] = wy;
			// ((float*)IMUFrame->Data)[2] = wz;

			// ((float*)IMUFrame->Data)[3] = ax;
			// ((float*)IMUFrame->Data)[4] = ay;
			// ((float*)IMUFrame->Data)[5] = az;

			// slamfile->AddFrame(IMUFrame);
            

		}

	}

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
        
    std::string fileName = "aqualoc_gt_traj_seq_0" + seqNumber + ".txt"; 
	std::ifstream inFile(rootFolder + "/aqualoc_gt_trajectories/" + fileName);

    if (!inFile) {
        std::cerr << "Unable to open file " + fileName;
        exit(1);  
    }

    // std::cerr << " Successfully located file " + fileName << std::endl;

	while (std::getline(inFile, line)){

		if (boost::regex_match(line,match,boost::regex("^([0-9.]+) ([-]?[0-9.]+) ([-]?[0-9.]+) ([-]?[0-9.]+) ([-]?[0-9.]+) ([-]?[0-9.]+) ([-]?[0-9.]+) ([-]?[0-9.]+)$"))) {

            // this data does not have a timestamp, only has frame ID. Should I be using the first column as timestamp?
            // it comes in 0.0, 0.5, 1.0 etc..
            // uint64_t timestamp = strtol(std::string(match[1]).c_str(), nullptr, 10);

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

			memcpy(gtFrame->Data,pose.data(),gtSensor->GetFrameSize(gtFrame));

			slamfile->AddFrame(gtFrame);

            // std::cerr << "Frame successfully added" << std::endl;

		} else {
			std::cerr << "Unknown line:" << line << std::endl;
            // float tx = 0;
			// float ty = 0;
			// float tz = 0;

			// float qx =  0;
			// float qy =  0;
			// float qz =  0;
            // float qw =  0;

            // // QUESTION: I transformed the data in the same way as EUROCMAV, is this correct?
			// Eigen::Matrix3f rotationMat = Eigen::Quaternionf(qw,qx,qy,qz).toRotationMatrix();
			// Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
			// pose.block(0,0,3,3) = rotationMat;
			// pose.block(0,3,3,1) << tx , ty , tz;


			// slambench::io::SLAMInMemoryFrame *gtFrame = new slambench::io::SLAMInMemoryFrame();
			// gtFrame->FrameSensor = gtSensor;
			// gtFrame->Data = malloc(gtSensor->GetFrameSize(gtFrame));

			// memcpy(gtFrame->Data,pose.data(),gtSensor->GetFrameSize(gtFrame));

			// slamfile->AddFrame(gtFrame);
		}


	}
}

    std::cerr << " Successfully created SLAMFile " << std::endl;
	
	return slamfile;

}





