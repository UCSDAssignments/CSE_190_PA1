#!/usr/bin/env python

from temperature_sensor import TempSensor
import rospy

tempSensor = TempSensor()
tempSensor.handle_activation_message(True)




