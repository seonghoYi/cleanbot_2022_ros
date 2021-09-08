import rospy

from sensor_msgs.msg import Image as rosImage
from sensor_msgs.msg import CompressedImage
from coral_ros_msgs.msg import BoundingBoxes
from coral_ros_msgs.msg import BoundingBox
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError 

from PIL import Image
from PIL import ImageDraw

from pycoral.adapters import common
from pycoral.adapters import detect
from pycoral.utils.dataset import read_label_file
from pycoral.utils.edgetpu import make_interpreter

import time



def draw_objects(draw, objs, labels):
    for obj in objs:
        bbox = obj.bbox
        draw.rectangle([(bbox.xmin, bbox.ymin), (bbox.xmax, bbox.ymax)],
                    outline='green')
        draw.text((bbox.xmin + 10, bbox.ymin + 10),
                '%s\n%.2f' % (labels.get(obj.id, obj.id), obj.score),
                fill='red')



cv_image = 0
def image_callback(img):
    
    global cv_image
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(img, "passthrough")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))



def main():
    global cv_image
    
    image_name = None
    model_file = None
    model_label = None
    
    rospy.init_node("coral_object_detector", anonymous=True)

    image_name = rospy.get_param('~input')
    model_file = rospy.get_param('~model_file')
    model_label = rospy.get_param('~model_label')

    image_sub = rospy.Subscriber(image_name, rosImage, image_callback)
    image_pub = rospy.Publisher("/coral_ros/detected_image", rosImage, queue_size=10)
    bounding_box_pub = rospy.Publisher("/coral_ros/bounding_boxes", BoundingBoxes, queue_size=10)


    labels = read_label_file(model_label)
    interpreter = make_interpreter(model_file)
    interpreter.allocate_tensors()
    
    #time.sleep(1)
    print("run")

    while not rospy.is_shutdown():
        start = time.time()
        
        if (str(type(cv_image)) == "<class 'int'>"):
            continue         
        image = Image.fromarray(cv_image)
        _, scale = common.set_resized_input(interpreter, image.size, lambda size: image.resize(size, Image.ANTIALIAS))
        interpreter.invoke()
        objs = detect.get_objects(interpreter, 0.4, scale)

        image = image.convert('RGB')
        draw_objects(ImageDraw.Draw(image), objs, labels)
        ImageDraw.Draw(image).text((10, 10), '%.1f' % (1/(time.time()-start)), fill='red')
        bounding_boxes = BoundingBoxes()

        for obj in objs:
            bounding_box = BoundingBox()
            bounding_box.xmin = obj.bbox.xmin
            bounding_box.ymin = obj.bbox.ymin
            bounding_box.xmax = obj.bbox.xmax
            bounding_box.ymax = obj.bbox.ymax
            bounding_box.probability = obj.score
            bounding_box.id = obj.id
            bounding_box.Class = labels.get(obj.id, obj.id)
            bounding_boxes.bounding_boxes.append(bounding_box)
        
        bounding_boxes.header.stamp = rospy.Time.now()
        bounding_box_pub.publish(bounding_boxes)

        msg = rosImage()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "/camera_link"
        msg.height = image.height
        msg.width = image.width
        msg.encoding = "rgb8"
        msg.is_bigendian = False
        msg.step = 3 * image.width
        msg.data = np.array(image).tobytes()
        image_pub.publish(msg)

        #print("FPS: {0}".format(1//(time.time()-start)))
    rospy.spin()

    


if __name__ == '__main__':
    main()
