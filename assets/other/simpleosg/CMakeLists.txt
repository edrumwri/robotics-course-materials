# setup some things initially
cmake_minimum_required (VERSION 2.6)
project (simple)
include (CheckIncludeFiles)
include (CheckLibraryExists)

# find libraries
find_package (osg REQUIRED)
find_package (osgViewer REQUIRED)
find_package (osgDB REQUIRED)
find_package (osgGA REQUIRED)
find_package (osgFX REQUIRED)
find_package (osgUtil REQUIRED)
find_package (OpenThreads REQUIRED)

# setup OSG includes and additional sources
include_directories (${OSG_INCLUDE_DIR})
include_directories (${OSGDB_INCLUDE_DIR})
include_directories (${OSGGA_INCLUDE_DIR})
include_directories (${OSGVIEWER_INCLUDE_DIR})
include_directories (${OPENTHREADS_INCLUDE_DIR})

add_executable(simple simple.cpp)
target_link_libraries(simple ${OSG_LIBRARIES})
target_link_libraries(simple ${OSGVIEWER_LIBRARIES})
target_link_libraries(simple ${OSGDB_LIBRARIES})
target_link_libraries(simple ${OSGGA_LIBRARIES})
target_link_libraries(simple ${OPENTHREADS_LIBRARIES})


