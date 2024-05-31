IMAGE_NAME:=sit-aw/sit-aw-anchoring
TAG:=0.1.0

CONTAINER:=ontology

SRC_WORKSPACE := $(shell pwd)/src
UTILS_WORKSPACE:= $(shell pwd)/utils
TESTS_WORKSPACE := $(shell pwd)/tests

default: all

all: start-docker

.build-docker: Dockerfile
	docker buildx build -t $(IMAGE_NAME):$(TAG) .
	@touch .build-docker

docker_exists=$(shell docker ps -a | grep -x $(CONTAINER))
docker_runs=$(shell docker ps | grep -x $(CONTAINER))

start-docker: .build-docker
	xhost +local:root 
ifeq ($(strip $(docker_exists)),)
	@echo "Create and start docker"
	docker run -it --rm --net=host --ipc=host --name $(CONTAINER) -v $(SRC_WORKSPACE):/root/ros2_ws/src -v $(TESTS_WORKSPACE):/tmp/tests -v $(UTILS_WORKSPACE):/tmp/utils -v /tmp/.X11-unix:/tmp/.X11-unix -v /dev/dri:/dev/dri -e DISPLAY=$(DISPLAY) $(IMAGE_NAME):$(TAG) /bin/bash
else ifeq ($(strip $(docker_runs)),)
	@echo "Restart docker"
	@docker start $(CONTAINER)
	@docker attach $(CONTAINER)
endif

build-docker:
	@rm -f .build-docker
	@make start-docker

stop-docker:
	docker stop $(CONTAINER)

clear-docker:
	docker rm $(CONTAINER)

join-docker:
	docker exec -it $(CONTAINER) /bin/bash

.PHONY: default all start-docker stop-docker join-docker
