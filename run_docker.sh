DOCKER_IMAGE=$1
NAME=$2
docker run -itd   --rm --privileged=true  --gpus all \
	-e NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,display \
	-e NVIDIA_VISIBLE_DEVICES=all \
	--user $(id -u):$(id -g) \
	-p 22501:5901 \
	-p 22263:22 \
	--user 0 \
	--shm-size 128g \
	-v /data1/liy/projects/ManiSkill:/workspace/robotwin_generation \
	--name $NAME \
	$DOCKER_IMAGE /bin/bash