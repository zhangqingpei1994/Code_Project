# Copyright (C) 2016 Luiz Carlos Vieira (http://www.luiz.vieira.nom.br)
#
# This file is part of FLAT.
#
# FLAT is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# FLAT is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.

# Minimum CMake version required 
cmake_minimum_required(VERSION 2.8.11)

# Project information
project (FLAT)

# Version data and header file
set (CT_VERSION_MAJOR 1)
set (CT_VERSION_MINOR 0)
set (CT_VERSION_PATCH 0)
set (CT_VERSION "${CT_VERSION_MAJOR}.${CT_VERSION_MINOR}.${CT_VERSION_PATCH}")
configure_file (
  "${PROJECT_SOURCE_DIR}/version.h.in"
  "${PROJECT_BINARY_DIR}/version.h"
)
include_directories("${PROJECT_BINARY_DIR}")

# Default build type if none was specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
    "MinSizeRel" "RelWithDebInfo")
endif()

# Qt configuration
set(CMAKE_INCLUDE_CURRENT_DIR ON) # Find includes in corresponding build directories
set(CMAKE_AUTOMOC ON) # Instruct CMake to run moc automatically when needed
set(CMAKE_AUTORCC ON) # Instruct CMake to run rcc automatically when needed
set(CMAKE_AUTOUIC ON) # Instruct CMake to run uic automatically when needed
find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Xml REQUIRED)

# Add all source and resource files
file(GLOB SRC src/*.cpp src/*.h)
file(GLOB RSC src/*.qrc)

# Add the executable
if(WIN32)
    add_executable(FLAT WIN32 ${SRC} ${RSC})
else()
    add_executable(FLAT ${SRC} ${RSC})
endif()
set_target_properties(FLAT PROPERTIES OUTPUT_NAME flat)
set_target_properties(FLAT PROPERTIES OUTPUT_NAME_DEBUG flatd)

# Set up the required libraries
target_link_libraries(FLAT Qt5::Core Qt5::Widgets Qt5::Xml)