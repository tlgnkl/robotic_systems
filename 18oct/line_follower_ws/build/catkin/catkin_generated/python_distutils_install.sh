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

echo_and_run cd "/root/line_follower_ws/src/catkin"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/root/line_follower_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/root/line_follower_ws/install/lib/python3/dist-packages:/root/line_follower_ws/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/root/line_follower_ws/build" \
    "/usr/bin/python3" \
    "/root/line_follower_ws/src/catkin/setup.py" \
    egg_info --egg-base /root/line_follower_ws/build/catkin \
    build --build-base "/root/line_follower_ws/build/catkin" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/root/line_follower_ws/install" --install-scripts="/root/line_follower_ws/install/bin"
