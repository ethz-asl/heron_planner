#!/bin/bash

# default options
DOCKER_IMAGE=hlp_dev
DOCKERFILE=dev.Dockerfile
CONTAINER_NAME=hlp_docker
BUILD=false
WORKSPACE=/local/home/harrisl/workspaces/hlp_ws

help()
{
    echo "Usage: run_docker.sh [ -d | --docker <image name> ]
               [ -b | --build <dockerfile name> ] [ -n | --name <container name> ]
               [ -w | --workspace <workspace path> ]
               [ -h | --help  ]"
    exit 2
}

SHORT=d:,b:,n:,w:,h
LONG=docker:,build:,name:,workspace:,help
OPTS=$(getopt -a -n run_docker --options $SHORT --longoptions $LONG -- "$@")
echo $OPTS

eval set -- "$OPTS"

while :
do
  case "$1" in
    -d | --docker )
      DOCKER="$2"
      shift 2
      ;;
    -b | --build )
      BUILD="true"
      DOCKERFILE="$2"
      shift 2
      ;;
    -n | --name )
      CONTAINER_NAME="$2"
      shift 2
      ;;
    -w | --workspace )
      WORKSPACE="$2"
      shift 2
      ;;
    -h | --help)
      help
      ;;
    --)
      shift;
      break
      ;;
    *)
      echo "Unexpected option: $1"
      help
      ;;
  esac
done

if [ "$BUILD" = true ]; then
     docker build -f $DOCKERFILE -t $DOCKER_IMAGE .
fi

XAUTH=/tmp/.docker.xauth

echo "Preparing Xauthority data..."
xauth_list=$(xauth nlist :0 | tail -n 1 | sed -e 's/^..../ffff/')
if [ ! -f $XAUTH ]; then
    if [ -n "$xauth_list" ]; then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

echo "Done."
echo ""
echo "Verifying file contents:"
file $XAUTH
echo "--> It should say \"X11 Xauthority data\"."
echo ""
echo "Permissions:"
ls -FAlh $XAUTH
echo ""
echo "Running Docker with ROS Networking..."

docker run -it --rm \
    --env="DISPLAY=$DISPLAY" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=${XAUTH:-/tmp/.docker.xauth}" \
    --net=host \
    --privileged \
    --name=$CONTAINER_NAME \
    --volume=$WORKSPACE:/root/hlp_ws \
    $DOCKER_IMAGE \
    bash

echo "Done."
