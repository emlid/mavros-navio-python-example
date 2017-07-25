# Mavros Navio Python example

## **Overview**

In this tutorial we will create a simple subscriber that obtains data from mavros node and prints it. Though mavros publishes a lot of different topics, we will examine /mavros/imu/data as for example. Handling another one will be slightly different, but general principles are the same.
We will consider the following steps:

- Creating a workspace
- Creating a package
- Writing a simple subscriber
- Running a node
## **Creating a catkin workspace**

If you aren’t familiar with catkin, you can look through [this page](http://wiki.ros.org/catkin/conceptual_overview). Let's create and build a catkin workspace:

    $ mkdir -p ~/imu_example/src
    $ cd ~/imu_example/
    $ catkin_make

This will produce CMakeLists.txt and many other important files in your home directory that we will inspect later.
Source your new setup file to overlay this workspace on top of your environment:

    $ source devel/setup.bash
## **Creating a package**

First of all change to the source space directory:

    $ cd ~/imu_example/src

Now use the **catkin_create_pkg** script to create a new package called 'imu_data' which depends on mavros, mavros_msgs and roscpp:

    $ catkin_create_pkg imu_data mavros sensor_msgs rospy

Next you need to build the packages in the catkin workspace:

    $ cd ~/imu_example
    $ catkin_make

To add the workspace to your ROS environment you need to source the generated setup file:

    $ . ~/imu_example/devel/setup.bash
## **Writing a simple subscriber**

First lets create a 'scripts' directory to store our Python scripts:

    $ cd ~/imu_example/src/imu_data
    $ mkdir scripts
    $ cd scripts

Create the imu_data.py and paste there the following code:

    #!/usr/bin/env python
    import rospy
    from sensor_msgs.msg import Imu
    
    
    def callback(data):
        rospy.loginfo(rospy.get_caller_id() + "\nlinear acceleration:\nx: [{}]\ny: [{}]\nz: [{}]"
        .format(data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z))
    
    
    def listener():
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("/mavros/imu/data", Imu, callback)
        rospy.spin()
    
    
    if __name__ == '__main__':
        listener()

Don't forget to make the node executable:

    $ chmod +x imu_data.py

**The code explained:**

    import rospy
    from sensor_msgs.msg import Imu

You need to import rospy if you are writing a ROS Node. The sensor_msgs.msg import is so that we can reuse the sensor_msgs/Imu message type.


    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/mavros/imu/data", Imu, callback)

This declares that your node subscribes to the chatter topic which is of type sensor_msgs.msgs.Imu. When new messages are received, callback is invoked with the message as the first argument.


    rospy.spin()

This simply keeps your node from exiting until the node has been shutdown.

**Building your node**
Go to your catkin workspace and run catkin_make:

    $ cd ~/imu_example
    $ catkin_make


## **Running your node**

Make sure that you understand what’s going on in [this tutorial.](https://docs.emlid.com/navio2/common/dev/ros/) To run your subscriber you need to have both roscore and mavros_node running. Enter the following:

    $ rosrun imu_data imu_data.py

If you did everything correctly, you will see the messages like this:

    [INFO] [WallTime: 1500991730.498287] /listener_10587_1500991729582
    linear acceleration:
    x: [0.26477955]
    y: [0.04903325]
    z: [9.8458766]

