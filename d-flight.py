#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix

# Define the area boundaries and altitude restriction
AREA_1_LATITUDE_MIN = 44.44132
AREA_1_LATITUDE_MAX = 44.44634
AREA_1_LONGITUDE_MIN = 8.83075
AREA_1_LONGITUDE_MAX = 8.84036
AREA_1_MIN_ALTITUDE = 100

AREA_2_LATITUDE_MIN = 44.40892
AREA_2_LATITUDE_MAX = 44.44132
AREA_2_LONGITUDE_MIN = 8.2319
AREA_2_LONGITUDE_MAX = 8.83075
AREA_2_MIN_ALTITUDE = 1000

AREA_3_LATITUDE_MIN = 44.45642	
AREA_3_LATITUDE_MAX = 44.45621
AREA_3_LONGITUDE_MIN = 8.23456
AREA_3_LONGITUDE_MAX = 8.5423
AREA_3_MIN_ALTITUDE = 50

AREA_4_LATITUDE_MIN = 44.45623
AREA_4_LATITUDE_MAX = 44.21354
AREA_4_LONGITUDE_MIN = 8.26865
AREA_4_LONGITUDE_MAX = 8.3695
AREA_4_MIN_ALTITUDE = 150

AREA_5_LATITUDE_MIN = 44.6562
AREA_5_LATITUDE_MAX = 44.6821
AREA_5_LONGITUDE_MIN = 8.2356
AREA_5_LONGITUDE_MAX = 8.3265
AREA_5_MIN_ALTITUDE = 120

AREA_6_LATITUDE_MIN = 44.68953
AREA_6_LATITUDE_MAX = 44.69532
AREA_6_LONGITUDE_MIN = 8.3659
AREA_6_LONGITUDE_MAX = 8.3895
AREA_6_MIN_ALTITUDE = 140



def gps_callback(msg):
    latitude = msg.latitude
    longitude = msg.longitude
    altitude = msg.altitude

    # Check if the drone is within the specified area
    if (AREA_1_LATITUDE_MIN <= latitude <= AREA_1_LATITUDE_MAX and
        AREA_1_LONGITUDE_MIN <= longitude <= AREA_1_LONGITUDE_MAX):
        # Check if the altitude is below the restricted minimum
        if altitude < AREA_1_MIN_ALTITUDE:
            rospy.logwarn("Please increase the altitude of the vehicle, restricted minimum altitude in this area is 100 meters")
         else : rospy.logwarn(" restricted minimum altitude in this area is 100 meters")

	# Check if the drone is within the specified area
    if (AREA_2_LATITUDE_MIN <= latitude <= AREA_2_LATITUDE_MAX and
        AREA_2_LONGITUDE_MIN <= longitude <= AREA_2_LONGITUDE_MAX):
        rospy.logwarn("Please leave the area immediately")

	# Check if the drone is within the specified area
    if (AREA_3_LATITUDE_MIN <= latitude <= AREA_3_LATITUDE_MAX and
        AREA_3_LONGITUDE_MIN <= longitude <= AREA_3_LONGITUDE_MAX):
        # Check if the altitude is below the restricted minimum
        if altitude < AREA_3_MIN_ALTITUDE:
            rospy.logwarn("Please increase the altitude of the vehicle, restricted minimum altitude in this area is 50 meters")
         else : rospy.logwarn(" Safe to fly, restricted minimum altitude in this area is 50 meters")


	# Check if the drone is within the specified area
    if (AREA_4_LATITUDE_MIN <= latitude <= AREA_4_LATITUDE_MAX and
        AREA_4_LONGITUDE_MIN <= longitude <= AREA_4_LONGITUDE_MAX):
        # Check if the altitude is below the restricted minimum
        if altitude < AREA_4_MIN_ALTITUDE:
            rospy.logwarn("Please increase the altitude of the vehicle, restricted minimum altitude in this area is 150 meters")
         else : rospy.logwarn("Safe to fly, restricted minimum altitude in this area is 150 meters")


# Check if the drone is within the specified area
    if (AREA_5_LATITUDE_MIN <= latitude <= AREA_5_LATITUDE_MAX and
        AREA_5_LONGITUDE_MIN <= longitude <= AREA_5_LONGITUDE_MAX):
        # Check if the altitude is below the restricted minimum
         if altitude < AREA_5_MIN_ALTITUDE:
            rospy.logwarn("Please increase the altitude of the vehicle, restricted minimum altitude in this area is 120 meters")
         else : rospy.logwarn("Safe to fly, restricted minimum altitude in this area is 120 meters")


	# Check if the drone is within the specified area
    if (AREA_6_LATITUDE_MIN <= latitude <= AREA_6_LATITUDE_MAX and
        AREA_6_LONGITUDE_MIN <= longitude <= AREA_6_LONGITUDE_MAX):
        # Check if the altitude is below the restricted minimum
         if altitude < AREA_6_MIN_ALTITUDE:
            rospy.logwarn("Please increase the altitude of the vehicle, restricted minimum altitude in this area is 140 meters")
         else : rospy.logwarn("Safe to fly, restricted minimum altitude in this area is 140 meters")

def airsim_drone_monitor():
    rospy.init_node('airsim_drone_monitor', anonymous=True)
    rospy.Subscriber('/airsim_node/drone/gps', NavSatFix, gps_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        airsim_drone_monitor()
    except rospy.ROSInterruptException:
        pass
