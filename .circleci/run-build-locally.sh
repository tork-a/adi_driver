#!/usr/bin/env bash

CIRCLE_TOKEN=28c9d6b21270b362d25f5ec6c11ee1f0fd340a7f

curl --user ${CIRCLE_TOKEN}: \
     --request POST \
     --form revision=4c8efba906fc2ccae1f29a72896ef716e8c3c76a \
     --form config=@config.yml \
     --form notify=false \
     https://circleci.com/api/v1.1/project/github/tork-a/adi_driver/tree/circleci2
