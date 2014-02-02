cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
#set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/Modules/" ${CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};C:/Users/Eric/Documents/GitHub/pcl/cmake/Modules") 
project(StrawberryPeaches)

MACRO(ADD_MSVC_PRECOMPILED_HEADER PrecompiledHeader PrecompiledSource SourcesVar)
  IF(MSVC)
    GET_FILENAME_COMPONENT(PrecompiledBasename ${PrecompiledHeader} NAME_WE)
    SET(PrecompiledBinary "${CMAKE_CURRENT_BINARY_DIR}/${PrecompiledBasename}.pch")
    SET(Sources ${${SourcesVar}})

    SET_SOURCE_FILES_PROPERTIES(${PrecompiledSource}
                                PROPERTIES COMPILE_FLAGS "/Yc\"${PrecompiledHeader}\" /Fp\"${PrecompiledBinary}\""
                                           OBJECT_OUTPUTS "${PrecompiledBinary}")
    SET_SOURCE_FILES_PROPERTIES(${Sources}
                                PROPERTIES COMPILE_FLAGS "/Yu\"${PrecompiledHeader}\" /FI\"${PrecompiledBinary}\" /Fp\"${PrecompiledBinary}\""
                                           OBJECT_DEPENDS "${PrecompiledBinary}")  
    # Add precompiled header to SourcesVar
    LIST(APPEND ${SourcesVar} ${PrecompiledSource})
  ENDIF(MSVC)
ENDMACRO(ADD_MSVC_PRECOMPILED_HEADER)

find_package(PCL 1.7.1)
#COMPONENTS common visualization features io)
#COMPONENTS common flann visualization surface filter)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

add_executable(Kinect2Mesh src/Kinect2Mesh/Kinect2Mesh.cpp)
target_link_libraries(Kinect2Mesh ${PCL_LIBRARIES})

#add_executable(ICP src/ICP/*.cpp)
#target_link_libraries(ICP ${PCL_LIBRARIES})

#add_executable(Registration src/Registration/*.cpp)
#target_link_libraries(Registration ${PCL_LIBRARIES})

#add_executable(Surface src/surface/*.cpp)
#target_link_libraries(Surface ${PCL_LIBRARIES})

#Set(SurfaceSegSources src/SurfaceSegmentation/*.cpp)
#ADD_MSVC_PRECOMPILED_HEADER("stdafx.h" "src/SurfaceSegmentation/stdafx.cpp" SurfaceSegSources)
#add_executable(SurfaceSeg ${SurfaceSegSources})
#target_link_libraries(SurfaceSeg ${PCL_LIBRARIES})
Set(CMAKE_BUILD_TYPE Debug)

Set(VisualizationSources src/Visualization/visualizer.cpp)
ADD_MSVC_PRECOMPILED_HEADER("stdafx.h" "src/Visualization/stdafx.cpp" VisualizationSources)
add_executable(Visualization ${VisualizationSources})
target_link_libraries(Visualization ${PCL_LIBRARIES})

Set(RegionGrowingSources src/RegionGrowing/region_growing_segmentation.cpp)
ADD_MSVC_PRECOMPILED_HEADER("stdafx.h" "src/RegionGrowing/stdafx.cpp" RegionGrowingSources)
add_executable(RegionGrowing ${RegionGrowingSources})
target_link_libraries(RegionGrowing ${PCL_LIBRARIES})

#${PCL_COMMON_LIBRARIES} ${PCL_FLANN_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES} ${PCL_SURFACE_LIBRARIES})
#${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_FLANN_LIBRARIES})

if(POLICY CMP0020)
  cmake_policy(SET CMP0020 NEW)
endif()
 
PROJECT(RenderWindowNoUiFile)
 
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})
 
if(${VTK_VERSION} VERSION_GREATER "6" AND VTK_QT_VERSION VERSION_GREATER "4")
  # Instruct CMake to run moc automatically when needed.
  set(CMAKE_AUTOMOC ON)
  find_package(Qt5Widgets REQUIRED QUIET)
else()
  find_package(Qt4 REQUIRED)
  include(${QT_USE_FILE})
endif()
 
include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})
 
file(GLOB UI_FILES src/qt/*.ui)
file(GLOB QT_WRAP src/qt/*.h)
file(GLOB CXX_FILES src/qt/*.cpp)
 
if(${VTK_VERSION} VERSION_GREATER "6" AND VTK_QT_VERSION VERSION_GREATER "4")
  qt5_wrap_ui(UISrcs ${UI_FILES} )
  # CMAKE_AUTOMOC in ON so the MocHdrs will be automatically wrapped.
  add_executable(RenderWindowNoUiFile MACOSX_BUNDLE
    ${CXX_FILES} ${UISrcs} ${QT_WRAP})
  qt5_use_modules(RenderWindowNoUiFile Core Gui)
  target_link_libraries(RenderWindowNoUiFile ${VTK_LIBRARIES} ${PCL_LIBRARIES})
else()
  QT4_WRAP_UI(UISrcs ${UI_FILES})
  QT4_WRAP_CPP(MOCSrcs ${QT_WRAP})
  add_executable(RenderWindowNoUiFile MACOSX_BUNDLE ${CXX_FILES} ${UISrcs} ${MOCSrcs})
 
  if(VTK_LIBRARIES)
    if(${VTK_VERSION} VERSION_LESS "6")
      target_link_libraries(RenderWindowNoUiFile ${VTK_LIBRARIES} QVTK ${PCL_LIBRARIES})
    else()
      target_link_libraries(RenderWindowNoUiFile ${VTK_LIBRARIES} ${PCL_LIBRARIES})
    endif()
  else()
    target_link_libraries(RenderWindowNoUiFile vtkHybrid QVTK vtkViews ${QT_LIBRARIES} ${PCL_LIBRARIES})
  endif()
endif()

if(POLICY CMP0020)
  cmake_policy(SET CMP0020 NEW)
endif()
 
PROJECT(3DEditor)
 
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})
 
if(${VTK_VERSION} VERSION_GREATER "6" AND VTK_QT_VERSION VERSION_GREATER "4")
  # Instruct CMake to run moc automatically when needed.
  set(CMAKE_AUTOMOC ON)
  find_package(Qt5Widgets REQUIRED QUIET)
else()
  find_package(Qt4 REQUIRED)
  include(${QT_USE_FILE})
endif()
 
include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})
 
file(GLOB UI_FILES src/qt/multInherit/*.ui)
file(GLOB QT_WRAP src/qt/multInherit/*.h)
file(GLOB CXX_FILES src/qt/multInherit/*.cxx)
 
if(${VTK_VERSION} VERSION_GREATER "6" AND VTK_QT_VERSION VERSION_GREATER "4")
  qt5_wrap_ui(UISrcs ${UI_FILES} )
  # CMAKE_AUTOMOC in ON so the MocHdrs will be automatically wrapped.
  add_executable(3DEditor MACOSX_BUNDLE
    ${CXX_FILES} ${UISrcs} ${QT_WRAP})
  qt5_use_modules(3DEditor Core Gui)
  target_link_libraries(3DEditor ${VTK_LIBRARIES} ${PCL_LIBRARIES})
else()
  QT4_WRAP_UI(UISrcs ${UI_FILES})
  QT4_WRAP_CPP(MOCSrcs ${QT_WRAP})
  add_executable(3DEditor MACOSX_BUNDLE ${CXX_FILES} ${UISrcs} ${MOCSrcs})
 
  if(VTK_LIBRARIES)
    if(${VTK_VERSION} VERSION_LESS "6")
      target_link_libraries(3DEditor ${VTK_LIBRARIES} QVTK ${PCL_LIBRARIES})
    else()
      target_link_libraries(3DEditor ${VTK_LIBRARIES} ${PCL_LIBRARIES})
    endif()
  else()
    target_link_libraries(3DEditor vtkHybrid QVTK vtkViews ${QT_LIBRARIES} ${PCL_LIBRARIES})
  endif()
endif()

project(openni_grabber)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (openni_grabber src/capture/openni_viewer_simple.cpp)
target_link_libraries (openni_grabber ${PCL_LIBRARIES})

project(point_cloud_compression)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## Compiler flags
if(CMAKE_COMPILER_IS_GNUCXX)
#    set(CMAKE_CXX_FLAGS "-g")        
#    set(CMAKE_CXX_FLAGS "-pg")        
endif()

add_executable (point_cloud_compression src/compression/point_cloud_compression.cpp)
target_link_libraries (point_cloud_compression ${PCL_LIBRARIES})

project(zlib_depth_compression)

find_package(PCL 1.2 REQUIRED)
find_package( ZLIB REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## Compiler flags
if(CMAKE_COMPILER_IS_GNUCXX)
#    set(CMAKE_CXX_FLAGS "-g")        
#    set(CMAKE_CXX_FLAGS "-pg")        
endif()

#file(GLOB 
set(ZLIB_KINECT_SRCS "src/zlibcompression/zpipe.cpp" "src/zlibcompression/zlibkinect.cpp")
message(STATUS "ZLIB_KINECT_SRCS=${ZLIB_KINECT_SRCS}")
add_executable (zlib_depth_compression ${ZLIB_KINECT_SRCS})
target_link_libraries (zlib_depth_compression ${PCL_LIBRARIES})
if ( ZLIB_FOUND )
	include_directories( ${ZLIB_INCLUDE_DIRS} )
	target_link_libraries( zlib_depth_compression ${ZLIB_LIBRARIES} )
endif( ZLIB_FOUND )

project(depth_client)

find_package(PCL 1.2 REQUIRED)
find_package( ZLIB REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## Compiler flags
if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-O3")        
#    set(CMAKE_CXX_FLAGS "-g")        
#    set(CMAKE_CXX_FLAGS "-pg")        
endif()

#file(GLOB 
set(DEPTH_CLIENT_SRCS "src/zlibcompression/client.cpp" "src/zlibcompression/zpipe.cpp" )
add_executable (depth_client ${DEPTH_CLIENT_SRCS})
target_link_libraries (depth_client ${PCL_LIBRARIES})
if ( ZLIB_FOUND )
	include_directories( ${ZLIB_INCLUDE_DIRS} )
	target_link_libraries( depth_client ${ZLIB_LIBRARIES} )
endif( ZLIB_FOUND )
