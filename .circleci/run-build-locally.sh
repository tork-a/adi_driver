#!/usr/bin/env bash

curl --user ${CIRCLE_TOKEN}: \
     --request POST \
     --form config=@config.yml \
     --form notify=false \
     https://circleci.com/api/v1.1/project/github/tork-a/adi_driver/tree/circleci2
#     --form revision=1c16e315b550a4cd3b11b8db69e1ba24c4f1a3bc \
