# Copyright 2012, 2013, 2014 CNRS-LAAS
#
# Author: Florent Lamiraux
#
# This file is part of test-hpp
# hpp-model-urdf is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# hpp-model-urdf is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Lesser Public License for more details.
# You should have received a copy of the GNU Lesser General Public License
# along with test-hpp  If not, see <http://www.gnu.org/licenses/>.

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

include(cmake/base.cmake)
include(cmake/boost.cmake)
include(cmake/eigen.cmake)

SET(PROJECT_NAME hpp-test)
SET(PROJECT_DESCRIPTION "Integration tests for Humanoid Path Planner platform.")
SET(PROJECT_URL "github.com/laas/hpp-test")

SETUP_PROJECT()

SEARCH_FOR_EIGEN()
# Activate hpp-util logging if requested
SET (HPP_DEBUG FALSE CACHE BOOL "trigger hpp-util debug output")
IF (HPP_DEBUG)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHPP_DEBUG")
ENDIF()

# Search for Boost.
# Boost.Test is used by the test suite.
SEARCH_FOR_BOOST()
ADD_REQUIRED_DEPENDENCY("hpp-util")
ADD_REQUIRED_DEPENDENCY("hpp-model-urdf")
ADD_REQUIRED_DEPENDENCY("hrp2_14_description")

ADD_SUBDIRECTORY(tests)

SETUP_PROJECT_FINALIZE()
