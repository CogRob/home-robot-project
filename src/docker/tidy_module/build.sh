#!/bin/bash
cp downloads/*.csv ../../tidy_module/data/
docker build -t homerobot_tidy_services:melodic .