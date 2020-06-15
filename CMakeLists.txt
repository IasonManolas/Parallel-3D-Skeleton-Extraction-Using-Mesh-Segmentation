cmake_minimum_required(VERSION 2.8)
project(ParallelSkeletonExtractor)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
if(CMAKE_VERSION VERSION_LESS "3.7.0")
    set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif()
#set(CMAKE_BUILD_TYPE "Debug")

#Add the project cmake scripts to the module path
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

#Download external dependencies NOTE: If the user has one of these libs it shouldn't be downloaded again.
include(${CMAKE_MODULE_PATH}/DownloadProject.cmake)
if (CMAKE_VERSION VERSION_LESS 3.2)
    set(UPDATE_DISCONNECTED_IF_AVAILABLE "")
else()
    set(UPDATE_DISCONNECTED_IF_AVAILABLE "UPDATE_DISCONNECTED 1")
endif()


#Add the project sources
file(GLOB SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

add_executable(${PROJECT_NAME} ${SOURCES})

#Glew
find_package(GLEW REQUIRED)
if (GLEW_FOUND)
    include_directories(${GLEW_INCLUDE_DIRS})
    link_libraries(${GLEW_LIBRARIES})
endif()

##CGAL library
download_project(PROJ               CGAL
                 GIT_REPOSITORY     https://github.com/CGAL/cgal.git
                 GIT_TAG            master
                 ${UPDATE_DISCONNECTED_IF_AVAILABLE}
)
#add_subdirectory(${CGAL_SOURCE_DIR})
set(CGAL_HEADER_ONLY true)
find_package(CGAL)
#target_include_directories(${PROJECT_NAME} PRIVATE ${CGAL_SOURCE_DIR})

download_project(PROJ               GLM
                 GIT_REPOSITORY     https://github.com/g-truc/glm.git
                 GIT_TAG            master
                 ${UPDATE_DISCONNECTED_IF_AVAILABLE}
)
target_include_directories(${PROJECT_NAME} PRIVATE ${GLM_SOURCE_DIR})

##Polyscope
#download_project(PROJ               POLYSCOPE
#                 GIT_REPOSITORY     https://github.com/nmwsharp/polyscope.git
#                 GIT_TAG	    master
#                 ${UPDATE_DISCONNECTED_IF_AVAILABLE}
#)
##target_include_directories(${PROJECT_NAME} PRIVATE ${POLYSCOPE_SOURCE_DIR})
#add_subdirectory(${POLYSCOPE_SOURCE_DIR})

#IGL
#download_project(PROJ               IGL
#                 GIT_REPOSITORY     https://github.com/libigl/libigl.git
#                 GIT_TAG	    master
#                 ${UPDATE_DISCONNECTED_IF_AVAILABLE}
#)
#set(LIBIGL_INCLUDE_DIR ${IGL_SOURCE_DIR}/include)
#find_package(LIBIGL REQUIRED QUIET)

#Assimp
download_project(PROJ               ASSIMP
                 GIT_REPOSITORY     https://github.com/assimp/assimp.git
                 GIT_TAG	        master
                 ${UPDATE_DISCONNECTED_IF_AVAILABLE}
)
add_subdirectory(${ASSIMP_SOURCE_DIR})
target_include_directories(${PROJECT_NAME} PRIVATE ${ASSIMP_SOURCE_DIR}/include)


##Eigen 3 NOTE: Eigen is required on the system the code is ran
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
target_include_directories(${PROJECT_NAME} PRIVATE "$ENV{EIGEN3_INCLUDE_DIR}")

##Qt

find_package(Qt5 COMPONENTS Core Widgets REQUIRED)

##Pthread NOTE: Not sure why this is needed
find_package (Threads REQUIRED)
target_link_libraries(${PROJECT_NAME}
    ${CMAKE_THREAD_LIBS_INIT}
)

#target_include_directories(${PROJECT_NAME}
#        PRIVATE ${vcglib_devel_SOURCE_DIR}
#        PRIVATE ${nlohmann_json_SOURCE_DIR}
#        PRIVATE ${GSL_SOURCE_DIR}/include
#        )

#Use C++17
if(MSVC)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /std:c++20")
else(MSVC)
        target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
endif(MSVC)

target_link_libraries(${PROJECT_NAME} Eigen3::Eigen Qt5::Widgets Qt5::Core CGAL::CGAL  GL GLEW assimp
)
#igl::core
#polyscope


