#!/bin/bash

roscd
ln -s $OLDPWD/point_downsample/ point_downsample
ln -s $OLDPWD/animation_host/ animation_host
ln -s $OLDPWD/blob_tracker/ blob_tracker
ln -s $OLDPWD/ola_dmx_driver/ ola_dmx_driver

#an ugly script but it'll do
