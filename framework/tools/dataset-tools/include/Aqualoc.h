/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#ifndef FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_AQUALOC_H_
#define FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_AQUALOC_H_

#include <ParameterManager.h>
#include <ParameterComponent.h>
#include <Parameters.h>

#include <io/sensor/Sensor.h>
#include <io/sensor/CameraSensor.h>
#include <io/sensor/DepthSensor.h>
#include <io/sensor/GroundTruthSensor.h>
#include <io/sensor/IMUSensor.h>
#include "../../dataset-tools/include/DatasetReader.h"

namespace slambench {

namespace io {

class AqualocReader :  public DatasetReader {

public :
	std::string aqualocInputDir, aqualocDatasetFolder;
	bool grey = true, rgb = true, depth = true, gt = true, imu = true, dist = true; // set the sensor parameters here 
	bool positive_focal = false;

	AqualocReader (std::string name) : DatasetReader(name) {
		this->addParameter(TypedParameter<std::string>("i",     "input-directory",       "path of the Aqualoc dataset directory",   &this->aqualocInputDir, NULL));
		this->addParameter(TypedParameter<std::string>("i",     "input-dataset",       "the path of the specific Aqualoc dataset to use",   &this->aqualocDatasetFolder, NULL));
		this->addParameter(TypedParameter<bool>("depth",     "depth",       "set to true or false to specify if the DEPTH stream need to be include in the slam file.",   &this->depth, NULL));
		this->addParameter(TypedParameter<bool>("gt",     "gt",       "set to true or false to specify if the GROUNDTRUTH POSE stream need to be include in the slam file.",   &this->gt, NULL));    
		this->addParameter(TypedParameter<bool>("imu",     "imu",       "include text about IMU.",   &this->imu, NULL));
		this->addParameter(TypedParameter<bool>("dist", "dist", "determine if the input dataset uses distortion or not", &this->dist, NULL));
	}

	SLAMFile* GenerateSLAMFile () ;

private :
	CameraSensor *rgb_sensor = nullptr;
	DepthSensor *depth_sensor = nullptr;
	CameraSensor *grey_sensor =  nullptr;
	GroundTruthSensor *gt_sensor =  nullptr;
	IMUSensor *imu_sensor =  nullptr;


};

}
}

#endif /* FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_AQUALOC_H_ */
