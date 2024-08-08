import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

# 初始化cv_bridge
bridge = CvBridge()

def image_callback(ros_image):
    # 尝试将ROS图像消息转换为OpenCV格式
    try:
        # 这里假设ROS图像是BGR8格式
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
        print(e)
        return
    
    # 现在您可以使用cv_image执行任何OpenCV操作
    # 例如，显示图像
    cv2.imshow("Image Window", cv_image)
    cv2.waitKey(3)

def main():
    # 初始化ROS节点
    rospy.init_node('image_converter', anonymous=True)
    
    # 订阅图像话题
    image_sub = rospy.Subscriber("ar3/cameral/image", Image, image_callback)
    
    # 循环等待消息
    rospy.spin()

if __name__ == '__main__':
    main()
