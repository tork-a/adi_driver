#!/bin/bash

set -e

# install latest hub 2017.08.04
git clone -b v2.3.0-pre10 https://github.com/github/hub.git
(cd hub; ./script/build; sudo cp bin/hub /usr/local/bin/)

echo "show hub version"
hub version

mkdir -p ~/.config/
echo "github.com:" > ~/.config/hub
echo "- user: 7675t" >> ~/.config/hub
echo "  oauth_token: $GITHUB_ACCESS_TOKEN" >> ~/.config/hub

files=''
for file in ~/catkin_ws/artifacts/*-$CIRCLE_TAG.pdf ~/catkin_ws/artifacts/*.deb ; do
    files="$files -a $file"
done

set -x
hub release create -p $files -m "$CIRCLE_TAG"$'\n\n'"Released on `date '+%Y/%m/%d %H:%M:%S'`" $CIRCLE_TAG
