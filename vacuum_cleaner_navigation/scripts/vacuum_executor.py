from sys import path
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped

border_section = [[(-0.033446669578552246, 0.4209034740924835),
                    (-0.033446669578552246, 0.4209034740924835),
                    (-0.5647101998329163, 0.4384326636791229), 
                    (-0.548678457736969, -0.5046083331108093), 
                    (0.016533464193344116, -0.4460331201553345), 
                    (-0.033446669578552246, 0.4209034740924835)], 
                   
                  [(0.9234755039215088, 0.5434647798538208), 
                  (0.0012318789958953857, 0.4600996971130371), 
                  (0.08640652894973755, -0.3921983242034912), 
                  (1.0174694061279297, -0.3537544906139374), 
                  (0.9249651432037354, 0.5271152257919312)]]

path_state = False
robot_state = False
suction_state = False

def state_callback(msg):
    global path_state
    path_state = msg.data

def point_iter(border_point):
    for point in border_point:
        yield point



rospy.init_node("vacuum_executor", anonymous=True)


point_pub = rospy.Publisher("clicked_point", PointStamped, queue_size=10)
suction_pub = rospy.Publisher("cmd_suction", Bool, latch=True, queue_size=10)
rospy.Subscriber("path_coverage/state", Bool, state_callback)

r = rospy.Rate(1.0)

while not rospy.is_shutdown():
    border = None
    if path_state == False and robot_state == False:
        print('a')
        border = border_section[0]
        suction_pub.publish(Bool(True))
    elif path_state == True and robot_state == False:
        path_state = False
        robot_state = True
        border = border_section[1]
    elif path_state == True and robot_state == True:
        print('b')
        suction_pub.publish(Bool(False))
        exit()

    
    
    msg = PointStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = rospy.Time()
    
    for point in border:
        print(point)
        msg.point.x = point[0]
        msg.point.y = point[1]
        point_pub.publish(msg)
        r.sleep()
    
    while not path_state:
        continue
rospy.spin()

