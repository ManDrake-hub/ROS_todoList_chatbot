#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/luigi/Scrivania/ROS_todoList_chatbot/cogrob_ws/src/pepper_nodes"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/luigi/Scrivania/ROS_todoList_chatbot/cogrob_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/luigi/Scrivania/ROS_todoList_chatbot/cogrob_ws/install/lib/python3/dist-packages:/home/luigi/Scrivania/ROS_todoList_chatbot/cogrob_ws/build/pepper_nodes/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/luigi/Scrivania/ROS_todoList_chatbot/cogrob_ws/build/pepper_nodes" \
    "/home/luigi/miniconda3/envs/CR/bin/python3" \
    "/home/luigi/Scrivania/ROS_todoList_chatbot/cogrob_ws/src/pepper_nodes/setup.py" \
     \
    build --build-base "/home/luigi/Scrivania/ROS_todoList_chatbot/cogrob_ws/build/pepper_nodes" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/luigi/Scrivania/ROS_todoList_chatbot/cogrob_ws/install" --install-scripts="/home/luigi/Scrivania/ROS_todoList_chatbot/cogrob_ws/install/bin"
