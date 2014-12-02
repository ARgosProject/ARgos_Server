#!/usr/bin/env bash

LD_PRELOAD="/usr/lib/x86_64-linux-gnu/libv4l/v4l2convert.so" ./argos_server 8888 eth0
