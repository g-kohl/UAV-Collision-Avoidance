import gz.transport13 as gz
from gz.msgs10.pose_pb2 import Pose

def teleport_uav():
    node = gz.Node()

    pose_msg = Pose()
    pose_msg.name = "x500_lidar_1"
    pose_msg.position.x = 0.0
    pose_msg.position.y = 0.0
    pose_msg.position.z = 3.0
    pose_msg.orientation.w = 1.0

    pose_bytes = pose_msg.SerializeToString()

    node.request_raw(
        "/world/default/set_pose",
        pose_bytes,
        "gz.msgs.Pose",
        "gz.msgs.Boolean",
        1000
    )

if __name__ == "__main__":
    teleport_uav()
