#!/bin/bash

git clone https://github.com/hanyazou/TelloPy.git tellopy
cd tellopy
pip install . --break-system-packages

cd ..
sudo rm -rf tellopy
