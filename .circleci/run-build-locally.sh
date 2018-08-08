#!/usr/bin/env bash

curl --user ${CIRCLE_TOKEN}: \
     --request POST \
     --form revision=7fe381ea9f78ccb0631e878bb99b1b969dd70ad2 \
     --form config=@config.yml \
     --form notify=false \
     https://circleci.com/api/v1.1/project/github/tork-a/adi_driver/tree/circleci2
