#!/bin/bash
xhost +
docker exec -u ubuntu -w /home/ubuntu nexarm /bin/zsh -c "source ~/.zshrc; cd ~/ros2_ws; ~/.sync_typerc.sh"
