#! /usr/bin/env bash

URL='https://github.com/diegocardozo97/DeepSpeech/releases/download/v0.1/language_model-mozilla.zip'
TARGET=language_model-mozilla.zip
echo "Download LibriSpeech model ..."
wget $URL

if [ $? -ne 0 ]; then
    echo "Fail to download LibriSpeech model!"
    exit 1
fi
unzip $TARGET
rm $TARGET

exit 0
