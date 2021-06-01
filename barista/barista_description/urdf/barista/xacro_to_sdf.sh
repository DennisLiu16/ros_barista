#!/usr/bin/env bash
echo "Cleaning sdf and urdf"
rm -rf barista_hexagons_asus_xtion_pro.urdf barista_hexagons_asus_xtion_pro.sdf
rosrun xacro xacro.py barista_hexagons_asus_xtion_pro.urdf.xacro > barista_hexagons_asus_xtion_pro.urdf
gz sdf -p barista_hexagons_asus_xtion_pro.urdf > barista_hexagons_asus_xtion_pro.sdf
echo "Generated SDF"
