from arm_composites_manufacturing_process import ProcessController
import rospy
import time


def main():
    rospy.init_node("pick_and_place_panel", anonymous=True)
    
    p=ProcessController(disable_ft=True)
    
    # Wait for payload messages to arrive
    time.sleep(0.5)
    
    rospy.loginfo("Begin panel pick and place script")
    p.pickup_prepare('leeward_mid_panel')
    p.pickup_lower()
    p.pickup_grab()
    p.pickup_raise()
    
    p.transport_payload('panel_nest_leeward_mid_panel_target')
    p.place_lower()
    p.place_set()
    p.place_raise()
    rospy.loginfo("End panel pick and place script")

if __name__ == '__main__':
    main()