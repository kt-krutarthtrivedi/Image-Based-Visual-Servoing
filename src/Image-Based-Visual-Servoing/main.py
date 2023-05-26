# Basic ROS 2 program to subscribe to real-time streaming 
# video from your built-in webcam
# This code is modified by Berk Calli from the following author.
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com

"""
Modified by Krutarth Ambarish Trivedi (ktrivedi@wpi.edu) for academic purpose.
"""
  
# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy  #Numpy
from std_msgs.msg import Float64MultiArray

feature_locations = []

def rotz(theta):
  """
  Calculate rotation around Z axis for the given angle

  Attributes
  ----------
  theta : double
    an angle of rotation

  Returns
  -------
  R : a numpy array
    Rotation Matrix
  """
  R = numpy.array([[numpy.cos(theta), -numpy.sin(theta)], [numpy.sin(theta), numpy.cos(theta)]])
  return R

def calculate_skew_matrix(vector):
  """
  Calculate a skew symmetric matrix for the given 3D vector

  Attributes
  ----------
  vector  : a numpy array

  Returns
  -------
  S : a numpy array
    A 3x3 Skew symmetric matrix
  """
  S = numpy.zeros([3,3]) 

  S[0,1] = -vector[2]
  S[0,2]= vector[1]
  S[1,2] = -vector[0]
  S[1,0] = vector[2]
  S[2,0] = -vector[1]
  S[2,1] = vector[0]

  return S

def calculate_robot_jacobian():
  """
  Calculate robot jacobian matrix 

  Attributes
  ----------
  none

  Returns
  -------
  J : a numpy array
    a robot jacobian matrix
  """
  #robot parameters
  w1 = numpy.array([0,0,1])
  w2 = numpy.array([0,0,1])
  r1 = numpy.array([0.90,0,0])
  r2 = numpy.array([0.45,0,0])

  v1 = numpy.matmul(calculate_skew_matrix(w1), r1)
  v2 = numpy.matmul(calculate_skew_matrix(w2), r2)

  J1 = numpy.append(v1, w1, axis = 0)

  J2 = numpy.append(v2, w2, axis = 0)

  J = numpy.transpose(numpy.vstack((J1, J2)))

  return J

def get_reference_features():
  """
  Get the reference features' position in the pixel frame.

  Attributes
  ----------
  none

  Returns
  -------
  Reference features position as a python list
  """
  # Red Mask Centroid=  ( 518 , 263 )
  # Green Mask Centroid=  ( 601 , 347 )
  # Blue Mask Centroid=  ( 518 , 347 )
  # Pink Mask Centroid=  ( 601 , 263 )

  return [518,263,601,347,518,347,601,263]

def convert_into_image_frame(image_size, features_pixel_frame):
  """
  Convert features' position from pixel frame to image frame

  Attributes
  ----------
  image_size : a python list
    current frame size
  features_pixel_frame  : a python list
    features represented in the pixel frame

  Returns
  -------
  features_image_frame  : a python list
    features represented in the image frame
  """
  center_image_frame = [image_size[0]/2, image_size[1]/2]
  features_image_frame = []
  for i in range(len(features_pixel_frame)):
    if (i % 2 == 0):
      features_image_frame.append(center_image_frame[0] - features_pixel_frame[i])
    else:
      features_image_frame.append(center_image_frame[1] - features_pixel_frame[i])

  return features_image_frame

def calculate_error_signal(current, reference):
  """
  calculate the error signal for the controller
  error = current feature - reference feature

  Attributes
  ----------
  current : a python list
    current detected features' position in image frame
  
  reference : a python list
    reference features' position in image frame 

  Returns
  -------
  error : a python list
    error signal

  """
  error = []
  for i in range(len(reference)):
    error.append(current[i] - reference[i])
  return error

def calculate_image_jacobian(error):
  """
  calculate the image jacobian
  Note: Please refer to the RBE490X reference material for more information
        on image jacobian.

  Attributes
  ----------
  error : a python list
    error signal

  Returns
  -------
  image_jacobina : a numpy array
    an image jacobian matrix
  """
  size = len(error)
  image_jacobian = numpy.zeros([size,2])

  f = 1
  Z = 1

  for rows in range(size):
    for columns in range(2):
      if(rows % 2 == 0):
        if columns == 0:
          image_jacobian[rows,columns] = -f/Z
        else:
          image_jacobian[rows,columns] = 0
      else:
        if columns == 0:
          image_jacobian[rows,columns] = 0
        else:
          image_jacobian[rows,columns] = -f/Z
  return image_jacobian

def calculate_camera_velocity(error):
  """
  calculate the robot velocity in the camera frame using an IBVS controller

  Attributes
  ----------
  error : a python list
    error signal

  Returns
  -------
  desired_camera_velocity : a numpy array
    desired velocity in the camera frame
  """
  lambda1 = -0.001 
  lambda2 = -0.001   
  gain = numpy.array([[lambda1, 0],
                      [0, lambda2]])

  image_jacobian = calculate_image_jacobian(error)
  desired_camera_veloctiy = -numpy.matmul(gain, numpy.matmul(numpy.linalg.pinv(numpy.array(image_jacobian)),error))
  return desired_camera_veloctiy

def detect_features(input_image_rgb):
  """
  detect the desired features from the current RGB frame

  Attributes
  ----------
  input_image_rgb : a RGB image

  Returns
  -------
  current_features  : a python list
    the current features - centroids of the detected circles
  """
  current_features = []

  image_rgb = input_image_rgb.copy()
  image_hsv = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2HSV)

  [red_masked, red_thresholded] = do_color_thresholding(image_hsv, image_rgb, 2)
  x,y = calculate_centroid(red_masked)
  current_features.append(x)
  current_features.append(y)
  cv2.circle(input_image_rgb, (x, y), 5, (255, 255, 255), -1)

  [green_masked, green_thresholded] = do_color_thresholding(image_hsv, image_rgb, 1)
  x,y = calculate_centroid(green_masked)
  current_features.append(x)
  current_features.append(y)
  cv2.circle(input_image_rgb, (x, y), 5, (255, 255, 255), -1)

  [blue_masked, blue_thresholded] = do_color_thresholding(image_hsv, image_rgb, 0)
  x,y = calculate_centroid(blue_masked)
  current_features.append(x)
  current_features.append(y)
  cv2.circle(input_image_rgb, (x, y), 5, (255, 255, 255), -1)

  [pink_masked, pink_thresholded] = do_color_thresholding(image_hsv, image_rgb, 3)
  x,y = calculate_centroid(pink_masked)
  current_features.append(x)
  current_features.append(y)
  cv2.circle(input_image_rgb, (x, y), 5, (255, 255, 255), -1)

  cv2.imshow("Current Features", input_image_rgb)

  return current_features

def do_color_thresholding(img_hsv, img_rgb, channel):
  """
  perform color thresholding

  Attributes
  ----------
  img_hsv : the current frame in HSV
  img_rgb : the current frame in RGB
  channel : the unique channel assigned to each feature

  Returns
  -------
  image_rgb : masked image
  threshold_rgb : thresholded image
  """
  image_hsv = img_hsv.copy()
  image_rgb = img_rgb.copy()
  lower_bound = numpy.array([[0,100,20], [40,100,20], [110,100,20], [140,100,20]])
  upper_bound = numpy.array([[10,255,255], [80,255,255], [130,255,255], [160,255,255]])
  mask = cv2.inRange(image_hsv, lower_bound[channel], upper_bound[channel])
  image_rgb = cv2.bitwise_and(image_rgb, image_rgb, mask =mask)
  gray_image = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2GRAY)
  ret,threshold_img = cv2.threshold(gray_image,10,255,0)
  return image_rgb, threshold_img

def calculate_centroid(masked_img):
  """
  calculate the centroids of the given masked image

  Attributes
  ----------
  masked_img  : a masked image

  Returns
  -------
  [x,y] : position of the centroid in x and y directions respectively in pixel frame
  """
  masked_image = masked_img.copy()
  gray_image = cv2.cvtColor(masked_image, cv2.COLOR_RGB2GRAY)
  ret,thresh = cv2.threshold(gray_image,10,255,0)
  arr = numpy.array(thresh)
  loc = numpy.array(numpy.where(arr == 255))
  x = int(numpy.mean(loc[1]))
  y = int(numpy.mean(loc[0]))
  return x, y

class ImageSubscriber(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_subscriber')
      
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.

    self.subscription = self.create_subscription(
      Image, 
      '/camera1/image_raw', 
      self.listener_callback, 
      10)
    self.subscription # prevent unused variable warning

    # Create the publisher. This publisher will publish an Image
    # to the video_frames topic. The queue size is 10 messages.
    self.publisher_ = self.create_publisher(Image, 'output_image', 10)

    #Create the publisher for sending joint variables back to gazebo
    self.publisher_joints = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)

      
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
   
  def listener_callback(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    self.get_logger().info('Receiving video frame')
 
    # Convert ROS Image message to OpenCV image
    current_frame = self.br.imgmsg_to_cv2(data)
 
    # PLACE YOUR CODE HERE. PROCESS THE CURRENT FRAME AND PUBLISH IT. IF YOU ARE HAVING DIFFICULTY PUBLISHING IT YOU CAN USE THE FOLLOWING LINES TO DISPLAY IT VIA OPENCV FUNCTIONS
    # cv2.imshow("output_image_BGR", current_frame)

    #Converting into the RGB color space
    image_rgb = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)
    cv2.imshow("Virutal Camera", image_rgb)

    current_features_pixel = detect_features(image_rgb)
    current_features_image = convert_into_image_frame(current_frame.shape, current_features_pixel)
    print("Current features in image: ")
    print(current_features_image)

    reference_features_pixel = get_reference_features()
    reference_features_image = convert_into_image_frame(current_frame.shape, reference_features_pixel)
    print("Reference features in image: ")
    print(reference_features_image)

    error_signal = calculate_error_signal(current_features_image, reference_features_image)
    print("Error = ", error_signal)

    for i in range(len(error_signal)):
      if (abs(error_signal[i]) > 1):
        keepServoing = True
        feature_locations.append(current_features_image)
        writeEnable = True
      else:
        keepServoing = False
        writeEnable = False
        break

    if (keepServoing == True):
      camera_velocity_camera_frame = calculate_camera_velocity(error_signal)

      rotation = rotz(90)
      
      camera_velocity_robot_frame = numpy.matmul(rotation, camera_velocity_camera_frame)
      camera_velocity_robot_frame = numpy.append(camera_velocity_robot_frame, [0,0,0,0])

      #transform camera velocity into the robot base frame and calculate the desired angles
      J = calculate_robot_jacobian()

      q_to_gazebo = Float64MultiArray()

      q = numpy.matmul(numpy.linalg.pinv(J), camera_velocity_robot_frame)
      print("Joint Variables = ", q)

      q_to_gazebo.data.append(q[0])
      q_to_gazebo.data.append(q[1])
      self.publisher_joints.publish(q_to_gazebo)

    else:
      if(writeEnable == True):
        file = open("trajectory.txt","w")
        file.write(str(feature_locations))
        file.close
        writeEnable = False

    cv2.waitKey(1)

    # Publish the image.
    # The 'cv2_to_imgmsg' method converts an OpenCV
    # image to a ROS 2 image message
    self.publisher_.publish(self.br.cv2_to_imgmsg(current_frame, encoding="bgr8"))
  
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  image_subscriber = ImageSubscriber()
  
  # Spin the node so the callback function is called.
  rclpy.spin(image_subscriber)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_subscriber.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
