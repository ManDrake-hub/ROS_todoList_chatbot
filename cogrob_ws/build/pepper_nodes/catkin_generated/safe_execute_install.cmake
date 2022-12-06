execute_process(COMMAND "/home/luigi/Scrivania/ROS_todoList_chatbot/cogrob_ws/build/pepper_nodes/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/luigi/Scrivania/ROS_todoList_chatbot/cogrob_ws/build/pepper_nodes/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
