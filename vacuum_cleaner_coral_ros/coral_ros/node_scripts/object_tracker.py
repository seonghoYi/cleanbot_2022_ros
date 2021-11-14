import rospy
import message_filters

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

from coral_ros_msgs.msg import BoundingBoxes
from vacuum_cleaner_msgs.msg import ObjectList
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge, CvBridgeError

import tf

import pyrealsense2 as rs


boxes = BoundingBoxes()
br = tf.TransformBroadcaster()
obj_list_pub = rospy.Publisher("coral_ros/object_list", ObjectList, queue_size=10)

def image_callback(img, info):
    global boxes, br
    bridge = CvBridge()
    intrinsics = rs.intrinsics()

    object_list = ObjectList()

    try:
        cv_image = bridge.imgmsg_to_cv2(img, "passthrough")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

    intrinsics.width = info.width
    intrinsics.height = info.height
    intrinsics.ppx = info.K[2]
    intrinsics.ppy = info.K[5]
    intrinsics.fx = info.K[0]
    intrinsics.fy = info.K[4]
    intrinsics.model = rs.distortion.none
    intrinsics.coeffs = [i for i in info.D]


    #sequence = 0
    for i in boxes.bounding_boxes:
        if i.probability > 0.45:
            obj_name = i.Class # + '-' + str(sequence)
            center_x = (i.xmax + i.xmin) // 2
            center_y = (i.ymax + i.ymin) // 2
            try:
                depth = cv_image[center_y, center_x] / 1000

                result = rs.rs2_deproject_pixel_to_point(intrinsics, [center_x, center_y], depth)

                obj_point = [result[2], -result[0], 0]

                stamped_point = PointStamped()
                stamped_point.header.frame_id = 'camera_color_frame'
                stamped_point.point.x = obj_point[0]
                stamped_point.point.y = obj_point[1]

                object_list.object_names.append(obj_name + '-' + str(int(depth * 100)))
                object_list.object_points.append(stamped_point)
                '''
                br.sendTransform(obj_point, 
                                tf.transformations.quaternion_from_euler(0,0,0),
                                rospy.Time.now(),
                                obj_name,
                                "camera_color_frame")
                '''
                #sequence = sequence + 1
            except:
                pass

    if len(object_list.object_names):
        obj_list_pub.publish(object_list)

def box_callback(msg):
    global boxes
    boxes = msg


def main():
    aligned_image = None
    camera_info = None
    bounding_boxes = None

    rospy.init_node("object_tracker", anonymous=True)

    aligned_image = rospy.get_param('~input_depth_image')
    camera_info = rospy.get_param('~camera_info')
    bounding_boxes = rospy.get_param('~bounding_boxes')
    image_sub = message_filters.Subscriber(aligned_image, Image)
    info_sub = message_filters.Subscriber(camera_info, CameraInfo)
    

    ts = message_filters.TimeSynchronizer([image_sub, info_sub], 10)
    ts.registerCallback(image_callback)

    box_sub = rospy.Subscriber(bounding_boxes, BoundingBoxes, box_callback)

    print('tracker running')

    while not rospy.is_shutdown():
        pass

    rospy.spin()





if __name__ == '__main__':
    main()