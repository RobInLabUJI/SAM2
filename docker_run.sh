#!/usr/bin/env bash

rocker --persist-image --ipc=host --x11 --nvidia --name sam2 \
  --port 8888:8888 --volume .:/home/user/SAM2 -- \
  ghcr.io/robinlabuji/sam2:latest bash

