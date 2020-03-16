PRINTF=printf
WGET:=wget
GET_REPLY:=read REPLY

ifeq ("${SBQUIET}","1")
GET_REPLY:=REPLY="y"
WGET:=wget -q
endif


####################################
#### SLAMBENCH  INFOS (DEFAULT) ####
####################################

infos :
	@${PRINTF} "\n*** SLAMBench is an evaluation framework for SLAM algorithms. *** "
	@${PRINTF} "\n\
	  (1) First, several dependencies are needed to compile SLAMBench and its use-cases.\n\
	      If you may have already them installed on your system,\n\
	      we propose you to download and install them automatically using the following command :\n\
	      - \033[1;32mmake deps\033[0m\n\n\
	      - \033[1;32mmake gcc_cuda (If you do not have a compatible toolchain for CUDA)\033[0m\n\n\
	  (2) Then, to compile the SLAMBench framework you just need to type :\n\
	      - \033[1;32mmake slambench\033[0m\n\n\
	  (3) To compile the use-cases as well :\n\
	      - \033[1;32mmake slambench APPS=\"kfusion;lsdslam\"\033[0m\n\n\
	      However these use-cases are not distributed with SLAMBench, you will have to download them using the command:\n\
	      - \033[1;32mmake usecases\033[0m\n\n\
	  (4) Finally to run SLAM system with a particular dataset we recommand to run:\n\
	      - \033[1;32mmake datasets\033[0m\n\n\
	"

####################################
#### SLAMBENCH BUILD            ####
####################################

ROOT_DIR=$(shell pwd)

#### Experience shows that space in paths can be problematic, we refuse this case.            
####################################################################################

SPACE=$(shell echo ' ')

ifneq (,$(findstring ${SPACE},$(ROOT_DIR)))
$(error SLAMBench should not be stored in a directory which path contains spaces, GNU Make does not like this kind of thing.)
endif

#### Various variables to fill       
###################################

DEPS_DIR=$(ROOT_DIR)/deps
REPOS_DIR=$(DEPS_DIR)/repos
DEPS_BUILD_DIR=$(DEPS_DIR)/build
DEPS_ARGS=
DEPS_ENV=


#### Where dependencies are
####################################



include framework/makefiles/*.make

ifneq ("$(USE_LLVM)","")
DEPS_ARGS+= -DUSE_LLVM=1
endif

ifneq ("$(wildcard ${OPENNI2_LIBRARY})","")
DEPS_ARGS+= -DOPENNI2_LIBRARY=${OPENNI2_LIBRARY}
endif

ifneq ("$(wildcard ${OPENNI2_INCLUDE_PATH})","")
DEPS_ARGS+= -DOPENNI2_INCLUDE_PATH=${OPENNI2_INCLUDE_PATH}
endif


ifneq ("$(wildcard ${CUDA_TOOLKIT_ROOT_DIR})","")
DEPS_ARGS+= -DCUDA_TOOLKIT_ROOT_DIR=${CUDA_TOOLKIT_ROOT_DIR}
endif

ifneq ("$(wildcard ${GCC7_COMPILER})","")
DEPS_ARGS+= -DCUDA_HOST_COMPILER=${GCC7_COMPILER}
endif

ifneq ("$(wildcard ${GCC5_COMPILER})","")
DEPS_ARGS+= -DCUDA_HOST_COMPILER=${GCC5_COMPILER}
endif

ifneq ("$(wildcard ${CERES_DIR})","")
DEPS_ARGS+= -DCERES_DIR=${CERES_DIR}
endif

ifneq ("$(wildcard ${GVARS_INCLUDE_DIR})","")
DEPS_ARGS+= -DGVARS_INCLUDE_DIR=${GVARS_INCLUDE_DIR}
endif


ifneq ("$(wildcard ${GVARS_LIBRARY})","")
DEPS_ARGS+= -DGVARS_LIBRARY=${GVARS_LIBRARY}
endif


ifneq ("$(wildcard ${CVD_INCLUDE_DIR})","")
DEPS_ARGS+= -DCVD_INCLUDE_DIR=${CVD_INCLUDE_DIR}
endif


ifneq ("$(wildcard ${CVD_LIBRARY})","")
DEPS_ARGS+= -DCVD_LIBRARY=${CVD_LIBRARY}
endif

ifneq ("$(wildcard ${OPENGV_INCLUDE_DIR})","")
DEPS_ARGS+= -DOPENGV_INCLUDE_DIR=${OPENGV_INCLUDE_DIR}
endif

ifneq ("$(wildcard ${OPENGV_LIBRARY})","")
DEPS_ARGS+= -DOPENGV_LIBRARY=${OPENGV_LIBRARY}
endif

ifneq ("$(wildcard ${BRISK_INCLUDE_DIR})","")
DEPS_ARGS+= -DBRISK_INCLUDE_PATH=${BRISK_INCLUDE_DIR}
endif

#ifneq ("$(wildcard ${PANGOLIN_DIR})","")
#DEPS_ENV+= Pangolin_DIR=${PANGOLIN_DIR}
#endif

ifneq ("$(wildcard ${EIGEN3_INCLUDE_DIR})","")
DEPS_ARGS+= -DEIGEN3_INCLUDE_DIR=${EIGEN3_INCLUDE_DIR} -DEIGEN3_VERSION_OK=ON 
DEPS_ARGS+= -DEIGEN_INCLUDE_DIR=${EIGEN3_INCLUDE_DIR} -DEIGEN_VERSION_OK=ON 
endif

ifneq ("$(wildcard ${TOON_INCLUDE_DIR})","")
DEPS_ARGS+= -DTOON_INCLUDE_PATH=${TOON_INCLUDE_DIR}
endif

ifneq ("$(wildcard ${OPENCV_DIR})","")
DEPS_ENV+= OpenCV_DIR=${OPENCV_DIR}
endif	

ifneq ("$(wildcard ${BRISK_DIR})","")
DEPS_ENV+= brisk_DIR=${BRISK_DIR}/
endif

ifneq ("$(wildcard ${PCL_DIR})","")
DEPS_ENV+= PCL_DIR=${PCL_DIR}/
endif

ifneq ("$(wildcard ${G2O_DIR})","")
DEPS_ENV+= G2O_ROOT=${G2O_DIR}/
endif

ifneq ("$(wildcard ${CUDA_INC_PATH})","")
DEPS_ENV+= CUDA_INC_PATH=${CUDA_INC_PATH}/
endif

ifneq ("$(wildcard ${SUITE_SPARSE_ROOT})","")
	DEPS_ARGS+= -DSUITE_SPARSE_ROOT=${SUITE_SPARSE_ROOT}
endif

ifneq ("$(wildcard ${FREEIMAGE_INCLUDE_PATH})","")
	DEPS_ARGS+= -DFreeImage_INCLUDE_PATH="${FREEIMAGE_INCLUDE_PATH}"
	DEPS_ARGS+= -DFreeImage_DYNAMIC_LIBRARY="${FREEIMAGE_DYNAMIC_LIBRARY}"
endif


#### Compilation targets
####################################


deps : 
	+make brisk
	+make ceres
	+make cvd
	+make eigen3
	+make flann
	+make freeimage
	+make g2o
	+make gvars
	+make opencv
	+make opengv
	+make opentuner
	+make pangolin
	+make pcl
	+make suitesparse
	+make toon


.PHONY: deps


build/Makefile : framework/CMakeLists.txt
	mkdir -p build/
	${DEPS_ENV} cmake -Bbuild -H.  -DAPPS="${APPS}"  ${DEPS_ARGS} -D"CMAKE_MODULE_PATH:PATH=${ROOT_DIR}/cmake_modules"


.PHONY: build/Makefile 

slambench: build/Makefile 
	$(MAKE) -C build $(MFLAGS)
	@${PRINTF} "\n"
	@${PRINTF} "=================================================================================================================\n"
	@${PRINTF} "The SLAMBench library should have been compiled, you will find binaries in build/bin, and libraries in build/lib.\n"
	@${PRINTF} "\n"
	@${PRINTF} "The list of current binaries is: \n"
	@${PRINTF} "\n"
	@${PRINTF} "Loaders available: \n"
	@${PRINTF}  "  - build/bin/benchmark_loader:    " ; if [ -f build/bin/benchmark_loader ] ; then ${PRINTF} "\033[1;32mFound\033[0m\n" ; else ${PRINTF} "\033[1;31mNot found (Missing dependencies?)\033[0m\n" ; fi
	@${PRINTF}  "  - build/bin/pangolin_loader:     " ; if [ -f build/bin/pangolin_loader ] ; then ${PRINTF} "\033[1;32mFound\033[0m\n" ; else ${PRINTF} "\033[1;31mNot found (Missing dependencies? did you try make pangolin)\033[0m\n" ; fi
	@${PRINTF} "\n\n"
	@${PRINTF} "Tools/Debugger available: \n"
	@${PRINTF}  "  - build/bin/pointcloud_aligner:  " ; if [ -f build/bin/pointcloud_aligner ] ; then ${PRINTF} "\033[1;32mFound\033[0m\n" ; else ${PRINTF} "\033[1;31mNot found (Missing dependencies (i.e. pcl)?)\033[0m\n" ; fi
	@${PRINTF}  "  - build/bin/dataset-generator:   " ; if [ -f build/bin/dataset-generator ] ; then ${PRINTF} "\033[1;32mFound\033[0m\n" ; else ${PRINTF} "\033[1;31mNot found (Missing dependencies?)\033[0m\n" ; fi
	@${PRINTF}  "  - build/bin/io-inspect-file:     " ; if [ -f build/bin/io-inspect-file ] ; then ${PRINTF} "\033[1;32mFound\033[0m\n" ; else ${PRINTF} "\033[1;31mNot found (Missing dependencies?)\033[0m\n" ; fi
	@${PRINTF}  "  - build/bin/io-readply:          " ; if [ -f build/bin/io-readply ] ; then ${PRINTF} "\033[1;32mFound\033[0m\n" ; else ${PRINTF} "\033[1;31mNot found (Missing dependencies?)\033[0m\n" ; fi
	@${PRINTF} "\n"
	@${PRINTF} "The list of current use-case libraries available is:\n"
	@${PRINTF} "\n"                                                                      
	@for f in `ls build/lib/lib*-library.so 2> /dev/null || ${PRINTF} Nothing` ; do ${PRINTF} $$f ; done
	@${PRINTF} "\n"            
	@${PRINTF} "The list of current filters available is:\n"
	@${PRINTF} "\n"                                                                      
	@for f in `ls build/lib/lib*-filter.so 2> /dev/null || ${PRINTF} Nothing` ; do ${PRINTF} $$f ; done
	@${PRINTF} "\n"
	@${PRINTF} "As a next step we suggest you to run \"make usecases\" or \"make slambench APPS=all\".\n"
	@${PRINTF} "\n"
	@${PRINTF} "=================================================================================================================\n"
	@${PRINTF} "\n"

framework/makefiles/benchmarks.makefile : framework/makefiles/download_benchmarks.py benchmarks/benchmarks.repos
	python $^ > $@

include framework/makefiles/benchmarks.makefile

datasets :
	@${PRINTF} "\n"
	@${PRINTF} "=================================================================================================================\n"
	@${PRINTF} "SLAMBench integrates tools to automatically generate files compatible with SLAMBench from existing datasets.\n"
	@${PRINTF} "\n"
	@${PRINTF} "The list of current dataset is:                                                                                 \n"
	@for f in `find datasets/ | grep [.]slam` ; do ${PRINTF} "   - $$f\n" ; done
	@${PRINTF} "\n"
	@${PRINTF} "If you do not find datasets in this list, you can use make to download them (make datasetslist). \n"
	@${PRINTF} "Here is a list of the datasets available.\n"
	@${PRINTF} "If you are using one of those dataset, \033[1;31mplease refer to their respective publications\033[0m:\n"
	@${PRINTF} "   - ICL-NUIM dataset [Handa et al, ICRA'14]: https://www.doc.ic.ac.uk/~ahanda/VaFRIC/iclnuim.html\n"
	@${PRINTF} "   - TUM RGB-D SLAM dataset [Sturm et al, IROS'12]: https://vision.in.tum.de/data/datasets/rgbd-dataset\n"
	@${PRINTF} "   - EuRoC MAV Dataset [Burri et al, IJRR'16]: https://projects.asl.ethz.ch/datasets/doku.php\n"
	@${PRINTF} "   - SVO sample dataset [Forster et al, ICRA 2014]: https://github.com/uzh-rpg/rpg_svo\n"
	@${PRINTF} "   - Aqualoc dataset [Ferrera et al, IJRR'19]: http://www.lirmm.fr/aqualoc\n"
	@${PRINTF} "=================================================================================================================\n"

datasetslist:
	@${PRINTF} "\n"
	@${PRINTF} "### ICL-NUIM Living Room\n"
	@${PRINTF} "\n"
	@${PRINTF} "make ./datasets/ICL_NUIM/living_room_traj0_loop.slam\n"
	@${PRINTF} "make ./datasets/ICL_NUIM/living_room_traj1_loop.slam\n"
	@${PRINTF} "make ./datasets/ICL_NUIM/living_room_traj2_loop.slam\n"
	@${PRINTF} "make ./datasets/ICL_NUIM/living_room_traj3_loop.slam\n"
	@${PRINTF} "\n"
	@${PRINTF} "\n"
	@${PRINTF} "### TUM Testing and Debugging\n"
	@${PRINTF} "\n"
	@${PRINTF} "make ./datasets/TUM/freiburg1/rgbd_dataset_freiburg1_xyz.slam\n"
	@${PRINTF} "make ./datasets/TUM/freiburg1/rgbd_dataset_freiburg1_rpy.slam\n"
	@${PRINTF} "make ./datasets/TUM/freiburg2/rgbd_dataset_freiburg2_xyz.slam\n"
	@${PRINTF} "make ./datasets/TUM/freiburg2/rgbd_dataset_freiburg2_rpy.slam\n"
	@${PRINTF} "\n"
	@${PRINTF} "\n"
	@${PRINTF} "### TUM Handheld SLAM\n"
	@${PRINTF} "\n"
	@${PRINTF} "make ./datasets/TUM/freiburg1/rgbd_dataset_freiburg1_360.slam\n"
	@${PRINTF} "make ./datasets/TUM/freiburg1/rgbd_dataset_freiburg1_floor.slam\n"
	@${PRINTF} "make ./datasets/TUM/freiburg1/rgbd_dataset_freiburg1_desk.slam\n"
	@${PRINTF} "make ./datasets/TUM/freiburg1/rgbd_dataset_freiburg1_desk2.slam\n"
	@${PRINTF} "make ./datasets/TUM/freiburg1/rgbd_dataset_freiburg1_room.slam\n"
	@${PRINTF} "\n"
	@${PRINTF} "make ./datasets/TUM/freiburg2/rgbd_dataset_freiburg2_360_hemisphere.slam\n"
	@${PRINTF} "make ./datasets/TUM/freiburg2/rgbd_dataset_freiburg2_360_kidnap.slam\n"
	@${PRINTF} "\n"
	@${PRINTF} "make ./datasets/TUM/freiburg2/rgbd_dataset_freiburg2_desk.slam\n"
	@${PRINTF} "make ./datasets/TUM/freiburg2/rgbd_dataset_freiburg2_desk_with_person.slam\n"
	@${PRINTF} "make ./datasets/TUM/freiburg2/rgbd_dataset_freiburg2_large_no_loop.slam\n"
	@${PRINTF} "make ./datasets/TUM/freiburg2/rgbd_dataset_freiburg2_large_with_loop.slam\n"
	@${PRINTF} "\n"
	@${PRINTF} "### TUM Robot SLAM\n"
	@${PRINTF} "\n"
	@${PRINTF} "make ./datasets/TUM/freiburg2/rgbd_dataset_freiburg2_pioneer_360.slam\n"
	@${PRINTF} "make ./datasets/TUM/freiburg2/rgbd_dataset_freiburg2_pioneer_slam.slam\n"
	@${PRINTF} "make ./datasets/TUM/freiburg2/rgbd_dataset_freiburg2_pioneer_slam2.slam\n"
	@${PRINTF} "make ./datasets/TUM/freiburg2/rgbd_dataset_freiburg2_pioneer_slam3.slam\n"
	@${PRINTF} "\n"
	@${PRINTF} "##### TUM Extra dataset\n"
	@${PRINTF} "\n"
	@${PRINTF} "## https://vision.in.tum.de/rgbd/dataset/freiburg3/rgbd_dataset_freiburg3_long_office_household.tgz\n"
	@${PRINTF} "\n"
	@${PRINTF} "\n"
	@${PRINTF} "### EuRoC MAV Machine Hall\n"
	@${PRINTF} "\n"
	@${PRINTF} "make ./datasets/EuRoCMAV/machine_hall/MH_01_easy/MH_01_easy.slam\n"
	@${PRINTF} "make ./datasets/EuRoCMAV/machine_hall/MH_02_easy/MH_02_easy.slam\n"
	@${PRINTF} "make ./datasets/EuRoCMAV/machine_hall/MH_03_medium/MH_03_medium.slam\n"
	@${PRINTF} "make ./datasets/EuRoCMAV/machine_hall/MH_04_difficult/MH_04_difficult.slam\n"
	@${PRINTF} "make ./datasets/EuRoCMAV/machine_hall/MH_05_difficult/MH_05_difficult.slam\n"
	@${PRINTF} "\n"
	@${PRINTF} "### EuRoC MAV Vicon Room\n"
	@${PRINTF} "\n"
	@${PRINTF} "make ./datasets/EuRoCMAV/vicon_room1/V1_01_easy/V1_01_easy.slam\n"
	@${PRINTF} "make ./datasets/EuRoCMAV/vicon_room1/V1_02_medium/V1_02_medium.slam\n"
	@${PRINTF} "make ./datasets/EuRoCMAV/vicon_room1/V1_03_difficult/V1_03_difficult.slam\n"
	@${PRINTF} "make ./datasets/EuRoCMAV/vicon_room2/V2_01_easy/V2_01_easy.slam\n"
	@${PRINTF} "make ./datasets/EuRoCMAV/vicon_room2/V2_02_medium/V2_02_medium.slam\n"
	@${PRINTF} "make ./datasets/EuRoCMAV/vicon_room2/V2_03_difficult/V2_03_difficult.slam\n"
	@${PRINTF} "\n"
	@${PRINTF} "\n"
	@${PRINTF} "### SVO test dataset\n"
	@${PRINTF} "\n"
	@${PRINTF} "make datasets/SVO/artificial.slam\n"
	@${PRINTF} "\n"
	@${PRINTF} "\n"
	@${PRINTF} "### Aqualoc dataset\n"
	@${PRINTF} "\n"
	@${PRINTF} "make ./datasets/Aqualoc/initial/sequence_1_no_bag.slam\n"
	@${PRINTF} "make ./datasets/Aqualoc/initial/sequence_2_no_bag.slam\n"
	@${PRINTF} "make ./datasets/Aqualoc/initial/sequence_3_no_bag.slam\n"
	@${PRINTF} "\n"
	@${PRINTF} "\n"
	@${PRINTF} "=================================================================================================================\n"
	@${PRINTF} "If you are using one of those dataset, \033[1;31mplease refer to their respective publications\033[0m:\n"
	@${PRINTF} "   - TUM RGB-D SLAM dataset [Sturm et al, IROS'12]: https://vision.in.tum.de/data/datasets/rgbd-dataset\n"
	@${PRINTF} "   - ICL-NUIM dataset [Handa et al, ICRA'14]: https://www.doc.ic.ac.uk/~ahanda/VaFRIC/iclnuim.html\n"
	@${PRINTF} "   - EuRoC MAV Dataset [Burri et al, IJJR'16]: https://projects.asl.ethz.ch/datasets/doku.php\n"
	@${PRINTF} "   - SVO sample dataset [Forster et al, ICRA 2014]: https://github.com/uzh-rpg/rpg_svo\n"
	@${PRINTF} "   - Aqualoc dataset [Ferrera et al, IJRR'19]: http://www.lirmm.fr/aqualoc\n"
	@${PRINTF} "=================================================================================================================\n"

.PHONY: slambench benchmarks benchmarkslist datasets datasetslist


####################################
#### DATA SET GENERATION        ####
####################################



#### EuRoCMAV
###############

./datasets/EuRoCMAV/%.zip :  # Example : $* = machine_hall/MH_01_easy/MH_01_easy
	mkdir -p $(@D)
	cd $(@D)  &&  ${WGET} "http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/$*.zip"


./datasets/EuRoCMAV/%.dir : ./datasets/EuRoCMAV/%.zip
	mkdir $@
	unzip $< -d $@

./datasets/EuRoCMAV/%.slam :  ./datasets/EuRoCMAV/%.dir 
	if [ ! -e ./build/bin/dataset-generator ] ; then make slambench ; fi
	./build/bin/dataset-generator -d eurocmav -i $</mav0 -o $@ -imu true -stereo true -gt true  

#### Aqualoc
###############

# ./datasets/Aqualoc/Test/InitialData/%.tar.gz :  # Example : $* = sequence_1_no_bag
# 	cd datasets/Aqualoc/Test 
# 	${WGET} "https://seafile.lirmm.fr/d/30fdf439c96343a3951b/files/?p=%2Faqualoc_gt_trajectories.tar.gz&dl=1" && tar xzf aqualoc_gt_trajectories.tar.gz
# 	${WGET} "https://seafile.lirmm.fr/d/30fdf439c96343a3951b/files/?p=%2Faqualoc_imu_cam_calib.txt&dl=1"
# 	${WGET} "https://seafile.lirmm.fr/d/30fdf439c96343a3951b/files/?p=%2Fcamchain-calib_aqualoc.yaml&dl=1"
# 	${WGET} "https://seafile.lirmm.fr/d/30fdf439c96343a3951b/files/?p=%2FMPU9250_imu_param.yaml&dl=1"
# 	mkdir -p $(@D)
# 	cd $(@D) &&  ${WGET} "https://seafile.lirmm.fr/d/30fdf439c96343a3951b/files/?p=%2F$*.tar.gz&dl=1"
	
# ./datasets/Aqualoc/Test/InitialData/%.dir : ./datasets/Aqualoc/Test/InitialData/%.tar.gz
# 	mkdir $@
# 	tar xzf $< -d $@

# ./datasets/Aqualoc/Test/InitialData/%.slam :  ./datasets/Aqualoc/Test/InitialData/%.dir 
# 	if [ ! -e ./build/bin/dataset-generator ] ; then make slambench ; fi,\
# 	./build/bin/dataset-generator -d aqualoc -idir datasets/AqualocTest/ idata datasets/Aqualoc/Test/$* -o $@   

#### TUM      
###############

./datasets/TUM/%.tgz :  # Example : $* = freiburg2/rgbd_dataset_freiburg2_desk 
	mkdir -p $(@D)
	cd $(@D)  &&  ${WGET} "http://vision.in.tum.de/rgbd/dataset/$*.tgz"

./datasets/TUM/%.dir : ./datasets/TUM/%.tgz
	mkdir $@
	tar xzf $< -C $@

./datasets/TUM/%.slam :  ./datasets/TUM/%.dir 
	if [ ! -e ./build/bin/dataset-generator ] ; then make slambench ; fi
	./build/bin/dataset-generator -d tum -i $</* -o $@ -grey true -rgb true -gt true -depth true -accelerometer true 

#### ICL-NUIM      
###############
datasets/ICL_NUIM/living-room.ply :  datasets/ICL_NUIM/living-room.ply.tar.gz
	cd datasets/ICL_NUIM  && tar xzf living-room.ply.tar.gz
	touch datasets/ICL_NUIM/living-room.ply # This is a fix to ensure not regenerating the file again because of file create date

datasets/ICL_NUIM/living-room.ply.tar.gz : 
	mkdir -p  datasets/ICL_NUIM
	cd datasets/ICL_NUIM  && ${WGET} "http://www.doc.ic.ac.uk/~ahanda/living-room.ply.tar.gz"

datasets/ICL_NUIM/%_loop.tgz : 
	mkdir -p  datasets/ICL_NUIM
	cd datasets/ICL_NUIM  && ${WGET} "http://www.doc.ic.ac.uk/~ahanda/$*_loop.tgz"

datasets/ICL_NUIM/%_loop.dir :  datasets/ICL_NUIM/%_loop.tgz
	mkdir -p $@
	tar xzf $< -C $@ 

datasets/ICL_NUIM/living_room_traj%_loop.slam : datasets/ICL_NUIM/living_room_traj%_loop.dir datasets/ICL_NUIM/living-room.ply
	if [ ! -e ./build/bin/dataset-generator ] ; then make slambench ; fi
	./build/bin/dataset-generator -d iclnuim -i $< -o $@ -ply  datasets/ICL_NUIM/living-room.ply -grey true -rgb true -gt true -depth true -pf true 

datasets/ICL_NUIM/living_room_traj%_loop_neg.slam : datasets/ICL_NUIM/living_room_traj%_loop.dir datasets/ICL_NUIM/living-room.ply
	if [ ! -e ./build/bin/dataset-generator ] ; then make slambench ; fi
	./build/bin/dataset-generator -d iclnuim -i $< -o $@ -ply  datasets/ICL_NUIM/living-room.ply -grey true -rgb true -gt true -depth true -pf false 


datasets/ICL_NUIM/%.gt.freiburg : 
	${PRINTF}  "Download ground truth trajectory...\n"
	mkdir -p datasets/ICL_NUIM/
	cd datasets/ICL_NUIM/  &&  if test -x livingRoom$*.gt.freiburg ; then ${PRINTF} "Done\n" ; else ${WGET} "http://www.doc.ic.ac.uk/~ahanda/VaFRIC/$*.gt.freiburg" ; fi

datasets/ICL_KLG/dyson_lab.klg :
	mkdir -p datasets/ICL_KLG/
	cd datasets/ICL_KLG/  &&  ${WGET} http://www.doc.ic.ac.uk/%7Esleutene/datasets/elasticfusion/dyson_lab.klg

#### SVO-artificial
###############
datasets/SVO/artificial.tar.gz:
	mkdir -p datasets/SVO/
	cd datasets/SVO && ${WGET} -O artificial.tar.gz "http://rpg.ifi.uzh.ch/datasets/sin2_tex2_h1_v8_d.tar.gz"

datasets/SVO/artificial.dir: ./datasets/SVO/artificial.tar.gz
	mkdir -p $@
	tar -xzf $< -C $@

datasets/SVO/artificial.slam: ./datasets/SVO/artificial.dir
	if [ ! -e ./build/bin/dataset-generator ] ; then make slambench ; fi
	./build/bin/dataset-generator -d svo -i $</sin2_tex2_h1_v8_d -o $@  


#### ORBSLAM Voc
#################

./benchmarks/orbslam2/src/original/Vocabulary/ORBvoc.txt : ./benchmarks/orbslam2/src/original/Vocabulary/ORBvoc.txt.tar.gz
	cd ./benchmarks/orbslam2/src/original/Vocabulary/ && tar -xf ORBvoc.txt.tar.gz

.PRECIOUS: ./datasets/TUM/%.tgz ./datasets/TUM/%.dir ./datasets/TUM/%.raw datasets/ICL_NUIM/living_room_traj%_loop.tgz ./datasets/TUM/%.raw datasets/ICL_NUIM/living_room_traj%_loop.dir datasets/ICL_NUIM/livingRoom%.gt.freiburg datasets/ICL_NUIM/living_room_traj%_loop.raw

####################################
####    BUILD/CLEAN TOOL        ####
####################################

#### CLEAN/CLEANALL TOOLS ####
clean :
	rm -rf build install android-build android-install docker/tmp/

cleandatasets :
	find datasets/ | grep [.]slam | xargs rm 2> /dev/null || true
	@${PRINTF} "Datasets Cleaned\n"

cleandeps :
	rm -rf deps

cleanall : clean cleandatasets cleandeps

.PHONY: clean cleandeps cleandatasets cleanall


####################################
####        DEMO PART           ####
####################################

demo-prepare :
	make slambench APPS=all
	make datasets/ICL_NUIM/living_room_traj2_loop.slam
	make datasets/EuRoCMAV/machine_hall/MH_01_easy/MH_01_easy.slam
	make datasets/TUM/freiburg2/rgbd_dataset_freiburg2_desk.slam

demo-lib :
	clear
	@${PRINTF} "                        \n"
	@${PRINTF} " =======================\n"
	@${PRINTF} " Pick UI                \n"
	@${PRINTF} "   *  depends on the purpose     \n"
	@${PRINTF} " =======================\n"
	@${PRINTF} "                        \n"
	@for d in `find ./build/bin/ -name *_loader ` ; do ${PRINTF} " $$d\n" ; done
	@${PRINTF} "                        \n"
	@${PRINTF} " =======================\n"
	@${PRINTF} " Pick dataset           \n"
	@${PRINTF} "   *  the slam format includes camera parameters \n"
	@${PRINTF} " =======================\n"
	@${PRINTF} "                        \n"
	@for d in `find datasets/ | grep [.]slam` ; do ${PRINTF} " -i $$d\n" ; done
	@${PRINTF} "                        \n"
	@${PRINTF} " =======================\n"
	@${PRINTF} " Pick libraries          \n"
	@${PRINTF} "   *  may requires parameters\n"
	@${PRINTF} "   *  do not need to be open source\n"
	@${PRINTF} " =======================\n"
	@${PRINTF} "                        \n"
	@for d in `find  build/lib/| grep [-]library[.]so` ; do ${PRINTF} " -load $$d\n" ; done
	@${PRINTF} "                        \n"
	@${PRINTF} "                        \n"
	@${PRINTF} " =======================\n"


####################################
####        TEST PART           ####
####################################

regression_test :
	make -C docker fastCI-docker-image # First test compilation of slambench and run kfusion
	make -C docker ubuntu-14.04-docker-image # Second compilation for embedded systems such as Odroid or TK1
# third run slambench for multiple datasets and check ATE and Memory usage

test :
	make -C docker fastCI-docker-image

