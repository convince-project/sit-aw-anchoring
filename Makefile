IMAGE_NAME:=sit-aw/sit-aw-anchoring
TAG:=0.2.0

CONTAINER:=sit-aw-anchoring-0.2.0

WS_VOLUME:=sit-aw-anchoring-workspace-humble

HISTFILE:=$(shell pwd)/.bash_history

USERNAME:=user

UID:=$(shell id -u)
GID:=$(shell id -g)

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
	docker run -it --rm -e UID=$(UID) -e GID=$(GID) --net=host --ipc=host --name $(CONTAINER) -v $(WS_VOLUME):/home/$(USERNAME)/sit-aw-anchoring/humble/src -v $(HISTFILE):/home/$(USERNAME)/.bash_history -v /tmp/.X11-unix:/tmp/.X11-unix -v /dev/dri:/dev/dri -e DISPLAY=$(DISPLAY) $(IMAGE_NAME):$(TAG) /bin/bash
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
	docker exec -u $(UID):$(GID) -it $(CONTAINER) /bin/bash

.PHONY: default all start-docker stop-docker join-docker
