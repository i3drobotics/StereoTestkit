cmake_minimum_required(VERSION 2.8.3)
set (CMAKE_CXX_STANDARD 11)
project(deimos_live)

find_package(OpenCV REQUIRED)

find_package(PCL 1.7 REQUIRED)

FIND_PACKAGE(ucoslam REQUIRED)

FIND_PACKAGE(PkgConfig)
PKG_CHECK_MODULES(PC_GLIB QUIET glib-2.0)

FIND_LIBRARY(
	GLIB_LIBRARIES
	NAMES glib-2.0
	HINTS ${PC_GLIB_LIBDIR}
	${PC_GLIB_LIBRARY_DIRS}
)

# Files in glib's main include path may include glibconfig.h, which,
# for some odd reason, is normally in $LIBDIR/glib-2.0/include.
GET_FILENAME_COMPONENT(_GLIB_LIBRARY_DIR ${GLIB_LIBRARIES} PATH)
FIND_PATH(
	GLIBCONFIG_INCLUDE_DIR
	NAMES glibconfig.h
	HINTS ${PC_LIBDIR} ${PC_LIBRARY_DIRS} ${_GLIB_LIBRARY_DIR}
	PATH_SUFFIXES glib-2.0/include
)

FIND_PATH(
	GLIB_INCLUDE_DIR
	NAMES glib.h
	HINTS ${PC_GLIB_INCLUDEDIR}
	${PC_GLIB_INCLUDE_DIRS}
	PATH_SUFFIXES glib-2.0
)

SET(GLIB_INCLUDE_DIRS ${GLIB_INCLUDE_DIR} ${GLIBCONFIG_INCLUDE_DIR})

link_directories(
	${PCL_LIBRARY_DIRS}
	)

add_definitions(${PCL_DEFINITIONS})

include_directories(
	include
	${CMAKE_CURRENT_SOURCE_DIR}/src/xunit
	${GLIB_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

add_library(xunit
  src/xunit/xunit_lib.c
  src/xunit/xunit_lib_cug.c
)

add_library(xunitTara
  src/xunit/xunit_lib_tara.h
  src/xunit/xunit_lib_tara.cpp
)

target_link_libraries(xunitTara ${udev_LIBRARIES} ${GLIB2_LIBRARIES})  
target_link_libraries(xunit ${udev_LIBRARIES})

add_executable(deimos_live src/deimos_live.cpp src/deimos_cam.cpp src/uvc_cam.cpp
				src/stereoMatcher/abstractStereoMatcher.cpp 
				src/stereoMatcher/matcherOpenCVBlock.cpp src/stereoMatcher/matcherOpenCVSGBM.cpp)

add_executable(deimos_ucoSLAM src/deimos_ucoSLAM.cpp 
				src/deimos_cam.cpp src/uvc_cam.cpp 
				src/stereoMatcher/abstractStereoMatcher.cpp 
				src/stereoMatcher/matcherOpenCVBlock.cpp src/stereoMatcher/matcherOpenCVSGBM.cpp
				src/PLYData.cpp
)

add_executable(mono_ucoSLAM src/mono_ucoSLAM.cpp 
				src/deimos_cam.cpp src/uvc_cam.cpp
				src/stereoMatcher/abstractStereoMatcher.cpp 
				src/stereoMatcher/matcherOpenCVBlock.cpp src/stereoMatcher/matcherOpenCVSGBM.cpp)

target_link_libraries(
	deimos_live 
	xunitTara 
	${GLIB2_LIBRARIES} -lv4l2 -lyaml-cpp -ludev -lglib-2.0 
	${OpenCV_LIBRARIES}
	${PCL_LIBRARIES}
)

target_link_libraries(
	mono_ucoSLAM 
	xunitTara 
	${GLIB2_LIBRARIES} -lv4l2 -lyaml-cpp -ludev -lglib-2.0 
	${OpenCV_LIBRARIES}
	${ucoslam_LIBS}
	${PCL_LIBRARIES}
)

target_link_libraries(
	deimos_ucoSLAM 
	xunitTara 
	${GLIB2_LIBRARIES} -lv4l2 -lyaml-cpp -ludev -lglib-2.0 
	${OpenCV_LIBRARIES}
	${ucoslam_LIBS}
	${PCL_LIBRARIES}
)