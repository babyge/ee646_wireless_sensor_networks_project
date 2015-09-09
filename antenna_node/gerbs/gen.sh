#!/bin/bash
rm antenna.zip
mv antenna_node-In1_Cu.gbr antenna_node-In1_Cu.G2L
mv antenna_node-In2_Cu.gbr antenna_node-In1_Cu.G3L
zip antenna.zip antenna*
