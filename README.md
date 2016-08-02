# Python RC Car Control
Python RC car tracking and path planning using OpenCV 3.0.0

## Installation
Clone the repository with 

`git clone https://github.com/zhangxingshuo/py-rc`

Make sure you install GQRX and GNURadio Companion following the instructions from [this repository](https://github.com/zhangxingshuo/gr-replay). 

## Usage
For a simple RC control platform, run

`python RCControl.py`

in CLI and control with the Q, W, E, A, D, Z, X, and C.

To use the overhead camera tracking, ensure that an overhead camera is mounted and on. Run

`python RCTracker.py`

in CLI. Click and drag to select the robot. Click again to set a destination.

## Credits
Harvey Mudd College Computer Science REU 2016

Ciante Jones: cjjones@hmc.edu

Chi-Yen Lee: johlee@hmc.edu

Andy Zhang: axzhang@hmc.edu

Professor Zach Dodds: zdodds@hmc.edu
