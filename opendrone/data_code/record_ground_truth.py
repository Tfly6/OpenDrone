"""Sample only Iris simulator pose; avoid recording every obstacle model.

GetModelState returns a simulator timestamp in its response header. Preserve
that stamp and log query duration separately; midpoint fallback is only used
if an older plugin supplies a zero timestamp.
"""
import rospy
from gazebo_msgs.srv import GetModelState
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64


def main():
    rospy.init_node("revision_ground_truth_recorder")
    publisher = rospy.Publisher("/flight_eval/ground_truth/odom", Odometry, queue_size=20)
    duration = rospy.Publisher("/flight_eval/ground_truth/query_duration", Float64, queue_size=20)
    rospy.wait_for_service("/gazebo/get_model_state", timeout=120)
    query = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    rate = rospy.Rate(30, reset=True)
    while not rospy.is_shutdown():
        start = rospy.Time.now()
        try:
            state = query("iris", "world")
            end = rospy.Time.now()
            if state.success and end >= start:
                msg = Odometry()
                msg.header.stamp = (state.header.stamp if state.header.stamp.to_sec() > 0
                                    else start + rospy.Duration((end-start).to_sec()/2))
                msg.header.frame_id = "world"
                msg.child_frame_id = "iris"
                msg.pose.pose = state.pose
                msg.twist.twist = state.twist
                publisher.publish(msg)
                duration.publish(Float64((end-start).to_sec()))
        except rospy.ServiceException as exc:
            rospy.logwarn_throttle(5, "Ground truth query failed: %s", exc)
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
