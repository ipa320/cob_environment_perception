all: cob_env_model 

SVN_REVISION = 
SVN_DIR = svntmp
#SVN_URL = svn://cob-server.ipa.fhg.de/cob/cob3_sw/trunk/Source/SLAM svn://cob-server.ipa.fhg.de/cob/cob3_sw/trunk/Source/Vision
SVN_URL = svn://cob-server.ipa.fhg.de/cob/cob3_sw/trunk/Source
include $(shell rospack find mk)/svn_checkout.mk

FILES_H=$(shell cd svntmp; ls SLAM/3DEnvReconstruction/*.h)
FILES_H+=$(shell cd svntmp; ls SLAM/3DEnvReconstruction/UI/*.h)
FILES_H+=$(shell cd svntmp; ls SLAM/DataAssociation/*.h)
FILES_H+=$(shell cd svntmp; ls SLAM/Filters/*.h)
FILES_H+=$(shell cd svntmp; ls SLAM/MapKdTree/*.h)
FILES_H+=$(shell cd svntmp; ls SLAM/MeasurementModels/*.h)
FILES_H+=$(shell cd svntmp; ls SLAM/Simulation/*.h)
FILES_H+=$(shell cd svntmp; ls SLAM/SystemModels/*.h)
FILES_H+=$(shell cd svntmp; ls Vision/Utilities/PointCloudRenderer.h)
FILES_H+=$(shell cd svntmp; ls Vision/Extern/Glut/*.h)
FILES_H+=$(shell cd svntmp; ls Vision/CameraSensors/CameraSensorsControlFlow.h)
FILES_H+=$(shell cd svntmp; ls Vision/Features/*.h)

#FILES_CPP=$(shell cd svntmp; ls *.cpp)
FILES_CPP=$(shell cd svntmp; ls SLAM/3DEnvReconstruction/*.cpp)
FILES_CPP+=$(shell cd svntmp; ls SLAM/3DEnvReconstruction/UI/*.cpp)
FILES_CPP+=$(shell cd svntmp; ls SLAM/DataAssociation/*.cpp)
FILES_CPP+=$(shell cd svntmp; ls SLAM/Filters/*.cpp)
FILES_CPP+=$(shell cd svntmp; ls SLAM/MapKdTree/*.cpp)
FILES_CPP+=$(shell cd svntmp; ls SLAM/MeasurementModels/*.cpp)
FILES_CPP+=$(shell cd svntmp; ls SLAM/Simulation/*.cpp)
FILES_CPP+=$(shell cd svntmp; ls SLAM/SystemModels/*.cpp)
FILES_CPP+=$(shell cd svntmp; ls Vision/Utilities/PointCloudRenderer.cpp)
FILES_CPP+=$(shell cd svntmp; ls Vision/Extern/Glut/*.cpp)
FILES_CPP+=$(shell cd svntmp; ls Vision/CameraSensors/CameraSensorsControlFlow.cpp)
FILES_CPP+=$(shell cd svntmp; ls Vision/Features/*.cpp)

cob_env_model: $(SVN_DIR)
	mkdir -p common/src
	mkdir -p common/include
	mkdir -p common/include/cob_env_model
	for i in $(FILES_H);do ln -sf ../../../svntmp/$$i common/include/cob_env_model/$(shell basename $$i);done
	for i in $(FILES_CPP);do ln -sf ../../svntmp/$$i common/src/$(shell basename $$i);done

include $(shell rospack find mk)/cmake.mk
