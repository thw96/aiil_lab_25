#!/bin/bash

# Get the current user
CURRENT_USER=$(whoami)

# Check if the current user is "husarion"
if [ "$CURRENT_USER" != "husarion" ]; then
  echo "This script can only be run by the user 'husarion'."
  exit 1
fi

# Define color codes
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BOLD='\033[1m'
NC='\033[0m' # No Color

# Stop the Docker containers if they're running
echo -e "${GREEN}[1/2]\r\nChecking if the required Docker Images are pulled ...${NC}"

# Define the docker-compose file
COMPOSE_FILE="/home/husarion/compose.yaml"

# This is a temporary solution allowing shared memory communication between 
# host and docker container. To be removed when user will be able to change this permission
# to something else than 0644 (https://github.com/eProsima/Fast-DDS/blob/master/thirdparty/boost/include/boost/interprocess/permissions.hpp#L100) 
# You need to start containers first, after that new files in /dev/shm/ are created. We need to change their permissions to 0666
export DOCKER_UID=$(id -u husarion)
export DOCKER_GID=$(id -g husarion)

# Service Image
SERVICE_IMAGES="rmit/aiil_workspace"

# Flag to track if any image is not pulled
IMAGE_NOT_FOUND=0

# Loop over each service image
for IMAGE in $SERVICE_IMAGES; do
    # Check if the image is pulled
    if [ -z "$(docker images -q $IMAGE)" ]; then
        echo -e "${YELLOW}Image ${BOLD}$IMAGE${NC}${YELLOW} is not pulled.${NC}"
        IMAGE_NOT_FOUND=1
    else
        echo -e "${GREEN}Image ${BOLD}$IMAGE${NC}${GREEN} is pulled.${NC}"
    fi
done

if [[ $IMAGE_NOT_FOUND -eq 1 ]]; then
    echo "Cannot find required image: ${SERVICE_IMAGES}"
    exit
fi

# Run bash
docker compose -f $COMPOSE_FILE run rmit_aiilworkspace bash
