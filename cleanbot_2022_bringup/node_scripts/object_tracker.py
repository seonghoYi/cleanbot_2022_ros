import rospy
import message_filters

from sensor_msgs.msg import Image as rosImage
from sensor_msgs.msg import CameraInfo

from cleanbot_2022_msgs.msg import BoundingBoxes
from cleanbot_2022_msgs.msg import ObjectList
from geometry_msgs.msg import PointStamped

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

import pyrealsense2 as rs


class ObjectTracker:
    def __init__(self):
        self.boxes = BoundingBoxes()
        self.is_boxes_subscribed = False

        self.image = rosImage()
        self.info = CameraInfo()
        self.is_image_subscribed = False

        self.filtered_img = np.zeros((240, 424))


        self.aligned_image = rospy.get_param('~input_depth_image', "/camera/aligned_depth_to_color/image_raw")
        self.camera_info = rospy.get_param('~camera_info', "/camera/aligned_depth_to_color/camera_info")
        self.bounding_boxes = rospy.get_param('~bounding_boxes', "/cleanbot_2022/bounding_boxes")
        self.probability_threshold = rospy.get_param('~probability_threshold', 0.45)
        self.update_hz = rospy.get_param("~update_hz", 10.0)


        self.image_sub = message_filters.Subscriber(self.aligned_image, rosImage)
        self.info_sub = message_filters.Subscriber(self.camera_info, CameraInfo)
        self.ts = message_filters.TimeSynchronizer([self.image_sub, self.info_sub], 10)
        self.ts.registerCallback(self.image_callback)
        self.box_sub = rospy.Subscriber(self.bounding_boxes, BoundingBoxes, self.box_callback)

        self.obj_list_pub = rospy.Publisher("cleanbot_2022/object_list", ObjectList, queue_size=10)
        self.filtered_depth_pub = rospy.Publisher("cleanbot_2022/aligned_depth_filtered", rosImage, queue_size=10)
        self.filtered_depth_info_pub = rospy.Publisher("cleanbot_2022/camera_info", CameraInfo, queue_size=10)


        rospy.loginfo("subscribe input_depth_image: %s", self.aligned_image)
        rospy.loginfo("subscribe camera_info: %s", self.aligned_image)
        rospy.loginfo("subscribe bounding_boxes: %s", self.aligned_image)
        rospy.loginfo("set probability_threshold: %f", self.probability_threshold)
        rospy.loginfo("set update_hz: %f", self.update_hz)

        rospy.loginfo("publish: /cleanbot_2022/object_list")
        rospy.loginfo("publish: /cleanbot_2022/aligned_depth_filtered")
        rospy.loginfo("publish: /cleanbot_2022/camera_info")


    def process(self):
        bridge = CvBridge()
        intrinsics = rs.intrinsics()
        object_list = ObjectList()

        info = self.info
        img = self.image


        original_header = img.header
        original_height = img.height
        original_width = img.width
        original_encoding = img.encoding
        original_bigendian = img.is_bigendian
        original_step = img.step




        if self.is_image_subscribed == True and self.is_boxes_subscribed == True:
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

            if len(self.boxes.bounding_boxes) == 0:
                self.filtered_img = cv_image
            for i in self.boxes.bounding_boxes:
                if i.probability > 0.45:
                    obj_name = i.Class
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

                        self.filtered_img = cv2.rectangle(cv_image, (i.xmin, i.ymin), (i.xmax, i.ymax), 0, -1)

                    except:
                        pass
                else:
                    self.filtered_img = cv_image






            filtered_ros_img = bridge.cv2_to_imgmsg(self.filtered_img, encoding="passthrough")
            filtered_ros_img.header = original_header
            filtered_ros_img.height = original_height
            filtered_ros_img.width = original_width
            filtered_ros_img.encoding = original_encoding
            filtered_ros_img.is_bigendian = original_bigendian
            filtered_ros_img.step = original_step
            info.header.stamp = rospy.Time.now()
            filtered_ros_img.header.stamp = rospy.Time.now()
            self.filtered_depth_pub.publish(filtered_ros_img)
            self.filtered_depth_info_pub.publish(info)

            if len(object_list.object_names):
                self.obj_list_pub.publish(object_list)

    def get_rate(self):
        return self.update_hz


    def box_callback(self, msg):
        self.boxes = msg
        self.is_boxes_subscribed = True


    def image_callback(self, img, info):
        self.image = img
        self.info = info
        self.is_image_subscribed = True

def main():
    rospy.init_node("cleanbot_2022_object_tracker", anonymous=True)

    object_tracker = ObjectTracker()

    rate = rospy.Rate(object_tracker.get_rate())

    print('object_tracker: tracker running.');

    while not rospy.is_shutdown():
        object_tracker.process()
        rate.sleep()

    rospy.spin()





if __name__ == '__main__':
    main()
