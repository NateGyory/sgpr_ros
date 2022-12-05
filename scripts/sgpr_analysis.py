#!/usr/bin/env python3

import rospy

def main():
    rospy.init_node('sgpr_analysis_node', anonymous=True)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
