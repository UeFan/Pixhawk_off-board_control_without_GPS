execute_process(COMMAND "/home/fanyue/catkin_ws/build/mavros-0.24.0/mavros/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/fanyue/catkin_ws/build/mavros-0.24.0/mavros/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
