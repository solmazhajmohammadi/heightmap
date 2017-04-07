project(test)
cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
find_package(PCL 1.7 REQUIRED )
find_package(OpenCV REQUIRED)
#add_definitions(-std=c++11)
include_directories(${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS} )
add_definitions(${PCL_DEFINITIONS}) 



add_executable (main 2DHeightMap.cpp )
target_link_libraries(main ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_LIBRARIES} ${PCL_KDTREE_LIBRARY} ${PCL_SEARCH_LIBRARY} ${PCL_SURFACE_LIBRARY} ${PCL_FEATURES_LIBRARY}  ${OpenCV_LIBS}) 


