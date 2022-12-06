# Install script for directory: /home/francesca/Scrivania/Intercorso/SECONDA_PARTE/ROS_todoList_chatbot/cogrob_ws/src/pepper_nodes

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/francesca/Scrivania/Intercorso/SECONDA_PARTE/ROS_todoList_chatbot/cogrob_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/francesca/Scrivania/Intercorso/SECONDA_PARTE/ROS_todoList_chatbot/cogrob_ws/install/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/francesca/Scrivania/Intercorso/SECONDA_PARTE/ROS_todoList_chatbot/cogrob_ws/install" TYPE PROGRAM FILES "/home/francesca/Scrivania/Intercorso/SECONDA_PARTE/ROS_todoList_chatbot/cogrob_ws/build/pepper_nodes/catkin_generated/installspace/_setup_util.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/francesca/Scrivania/Intercorso/SECONDA_PARTE/ROS_todoList_chatbot/cogrob_ws/install/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/francesca/Scrivania/Intercorso/SECONDA_PARTE/ROS_todoList_chatbot/cogrob_ws/install" TYPE PROGRAM FILES "/home/francesca/Scrivania/Intercorso/SECONDA_PARTE/ROS_todoList_chatbot/cogrob_ws/build/pepper_nodes/catkin_generated/installspace/env.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/francesca/Scrivania/Intercorso/SECONDA_PARTE/ROS_todoList_chatbot/cogrob_ws/install/setup.bash;/home/francesca/Scrivania/Intercorso/SECONDA_PARTE/ROS_todoList_chatbot/cogrob_ws/install/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/francesca/Scrivania/Intercorso/SECONDA_PARTE/ROS_todoList_chatbot/cogrob_ws/install" TYPE FILE FILES
    "/home/francesca/Scrivania/Intercorso/SECONDA_PARTE/ROS_todoList_chatbot/cogrob_ws/build/pepper_nodes/catkin_generated/installspace/setup.bash"
    "/home/francesca/Scrivania/Intercorso/SECONDA_PARTE/ROS_todoList_chatbot/cogrob_ws/build/pepper_nodes/catkin_generated/installspace/local_setup.bash"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/francesca/Scrivania/Intercorso/SECONDA_PARTE/ROS_todoList_chatbot/cogrob_ws/install/setup.sh;/home/francesca/Scrivania/Intercorso/SECONDA_PARTE/ROS_todoList_chatbot/cogrob_ws/install/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/francesca/Scrivania/Intercorso/SECONDA_PARTE/ROS_todoList_chatbot/cogrob_ws/install" TYPE FILE FILES
    "/home/francesca/Scrivania/Intercorso/SECONDA_PARTE/ROS_todoList_chatbot/cogrob_ws/build/pepper_nodes/catkin_generated/installspace/setup.sh"
    "/home/francesca/Scrivania/Intercorso/SECONDA_PARTE/ROS_todoList_chatbot/cogrob_ws/build/pepper_nodes/catkin_generated/installspace/local_setup.sh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/francesca/Scrivania/Intercorso/SECONDA_PARTE/ROS_todoList_chatbot/cogrob_ws/install/setup.zsh;/home/francesca/Scrivania/Intercorso/SECONDA_PARTE/ROS_todoList_chatbot/cogrob_ws/install/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/francesca/Scrivania/Intercorso/SECONDA_PARTE/ROS_todoList_chatbot/cogrob_ws/install" TYPE FILE FILES
    "/home/francesca/Scrivania/Intercorso/SECONDA_PARTE/ROS_todoList_chatbot/cogrob_ws/build/pepper_nodes/catkin_generated/installspace/setup.zsh"
    "/home/francesca/Scrivania/Intercorso/SECONDA_PARTE/ROS_todoList_chatbot/cogrob_ws/build/pepper_nodes/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/francesca/Scrivania/Intercorso/SECONDA_PARTE/ROS_todoList_chatbot/cogrob_ws/install/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/francesca/Scrivania/Intercorso/SECONDA_PARTE/ROS_todoList_chatbot/cogrob_ws/install" TYPE FILE FILES "/home/francesca/Scrivania/Intercorso/SECONDA_PARTE/ROS_todoList_chatbot/cogrob_ws/build/pepper_nodes/catkin_generated/installspace/.rosinstall")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/francesca/Scrivania/Intercorso/SECONDA_PARTE/ROS_todoList_chatbot/cogrob_ws/build/pepper_nodes/catkin_generated/safe_execute_install.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pepper_nodes/srv" TYPE FILE FILES
    "/home/francesca/Scrivania/Intercorso/SECONDA_PARTE/ROS_todoList_chatbot/cogrob_ws/src/pepper_nodes/srv/ExecuteJS.srv"
    "/home/francesca/Scrivania/Intercorso/SECONDA_PARTE/ROS_todoList_chatbot/cogrob_ws/src/pepper_nodes/srv/LoadUrl.srv"
    "/home/francesca/Scrivania/Intercorso/SECONDA_PARTE/ROS_todoList_chatbot/cogrob_ws/src/pepper_nodes/srv/Rest.srv"
    "/home/francesca/Scrivania/Intercorso/SECONDA_PARTE/ROS_todoList_chatbot/cogrob_ws/src/pepper_nodes/srv/Text2Speech.srv"
    "/home/francesca/Scrivania/Intercorso/SECONDA_PARTE/ROS_todoList_chatbot/cogrob_ws/src/pepper_nodes/srv/WakeUp.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pepper_nodes/cmake" TYPE FILE FILES "/home/francesca/Scrivania/Intercorso/SECONDA_PARTE/ROS_todoList_chatbot/cogrob_ws/build/pepper_nodes/catkin_generated/installspace/pepper_nodes-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/francesca/Scrivania/Intercorso/SECONDA_PARTE/ROS_todoList_chatbot/cogrob_ws/devel/.private/pepper_nodes/include/pepper_nodes")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/francesca/Scrivania/Intercorso/SECONDA_PARTE/ROS_todoList_chatbot/cogrob_ws/devel/.private/pepper_nodes/share/roseus/ros/pepper_nodes")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/francesca/Scrivania/Intercorso/SECONDA_PARTE/ROS_todoList_chatbot/cogrob_ws/devel/.private/pepper_nodes/share/common-lisp/ros/pepper_nodes")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/francesca/Scrivania/Intercorso/SECONDA_PARTE/ROS_todoList_chatbot/cogrob_ws/devel/.private/pepper_nodes/share/gennodejs/ros/pepper_nodes")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/francesca/Scrivania/Intercorso/SECONDA_PARTE/ROS_todoList_chatbot/cogrob_ws/devel/.private/pepper_nodes/lib/python3/dist-packages/pepper_nodes")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/francesca/Scrivania/Intercorso/SECONDA_PARTE/ROS_todoList_chatbot/cogrob_ws/devel/.private/pepper_nodes/lib/python3/dist-packages/pepper_nodes")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/francesca/Scrivania/Intercorso/SECONDA_PARTE/ROS_todoList_chatbot/cogrob_ws/build/pepper_nodes/catkin_generated/installspace/pepper_nodes.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pepper_nodes/cmake" TYPE FILE FILES "/home/francesca/Scrivania/Intercorso/SECONDA_PARTE/ROS_todoList_chatbot/cogrob_ws/build/pepper_nodes/catkin_generated/installspace/pepper_nodes-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pepper_nodes/cmake" TYPE FILE FILES
    "/home/francesca/Scrivania/Intercorso/SECONDA_PARTE/ROS_todoList_chatbot/cogrob_ws/build/pepper_nodes/catkin_generated/installspace/pepper_nodesConfig.cmake"
    "/home/francesca/Scrivania/Intercorso/SECONDA_PARTE/ROS_todoList_chatbot/cogrob_ws/build/pepper_nodes/catkin_generated/installspace/pepper_nodesConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pepper_nodes" TYPE FILE FILES "/home/francesca/Scrivania/Intercorso/SECONDA_PARTE/ROS_todoList_chatbot/cogrob_ws/src/pepper_nodes/package.xml")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/francesca/Scrivania/Intercorso/SECONDA_PARTE/ROS_todoList_chatbot/cogrob_ws/build/pepper_nodes/gtest/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/francesca/Scrivania/Intercorso/SECONDA_PARTE/ROS_todoList_chatbot/cogrob_ws/build/pepper_nodes/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
