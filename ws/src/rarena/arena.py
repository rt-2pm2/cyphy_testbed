#!/usr/bin/env python

import paho.mqtt.client as mqtt
import rospy
import os
import sys
import time
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), './')))

from commander_interface.srv import GoTo, Land
from guidance.srv import GenImpTrajectoryAuto

from geometry_msgs.msg import PoseStamped

import numpy as np


###### HELPERS #####
def posFromPoseMsg(pose_msg):
    pos = np.array([pose_msg.pose.position.x, 
                pose_msg.pose.position.y, 
                pose_msg.pose.position.z])
    return pos

def quatFromPoseMsg(pose_msg):
    quat = np.array([pose_msg.pose.orientation.x, 
                     pose_msg.pose.orientation.y,
                     pose_msg.pose.orientation.z,
                     pose_msg.pose.orientation.w])
    return quat 


################# ROS ARENA CLASS #####################
class RArenaClass(object):
    """
    Bridge class between Ros and Arena.
    """
    def __init__(self, client, scene, name, id, shape="cube", color="#AAAAAA", animate=False):

        self.client = client
        self.scene = scene
        self.name = name
        self.id = id
        self.shape = shape
        self.color = color
        self.base_topic = self.scene + "/"  + self.shape + "_{}".format(self.id) 
        self.text = name
        self.text_topic = self.scene + "/"  + "text"+ "_{}".format(self.id) 
        self.animate = animate

        self.registerCallbacks()
        self.initArenaObject()

    def initArenaObject(self):
        pass

    def registerCallbacks(self):
        pass

    def on_click_input(self, client, userdata, msg):
        pass

    def remove(self):
        self.client.publish(self.base_topic, "", retain=True)

    def __del__(self):
        self.remove()

    def set_color(self, new_color):
        self.color = new_color

    def start_animation(self):  
        self.animate = True

    def stop_animation(self):
        self.animate = False
        self.client.publish(self.base_topic + "/animation", "", retain=True)



###### NODE ARENA CLASS ####### 
class NodeArenaClass(RArenaClass):
    def __init__(self, client, scene, name, id, shape="cube", color="#AAAAAA", animate=False, transparent=False, 
        pos=[0,1,0], quat=[0,0,0,0], scale=[0.5, 0.5, 0.5], pose_source=None, show_text=True, offset=[0,0.5,0]):

        self.pos = pos
        self.quat = quat
        self.scale = scale
        self.pose_source = pose_source

        self.show_text = show_text
        self.text_quat = [0,0,0,1]
        self.text_scale = [0.08,0.08,0.08]
        self.offset = offset
        self.transparent = transparent
        self.opacity = 0.5

        super(NodeArenaClass, self).__init__(client, scene, name, id, shape=shape, color=color)

        print("Created Arena Node Object")


    def initArenaObject(self):
        #                                                      x  y  z  qx qy qz qw sx sy sz col
        self.message = self.shape + "_{}".format(self.id)  + ",{},{},{},{},{},{},{},{},{},{},{},on"
        #                           x  y  z  qx qy qz qw sx sy sz txt
        self.text_message = "text" + "_{}".format(self.id) + ",{},{},{},{},{},{},{},{},{},{},{},on"

        # Redraw object
        self.remove()
        self.draw()

        # Enable click listener for object (allows it to be clickable)
        self.client.publish(self.base_topic + "/click-listener", "enable", retain=True)
        self.client.subscribe(self.base_topic + "/mousedown")
        print("Subscribed to MQTT: " + self.base_topic + "/mousedown")
        self.client.message_callback_add(self.base_topic + "/mousedown", self.on_click_input)
        
        if self.transparent:
            self.client.publish(self.base_topic + "/material", "transparent: true; opacity: {}".format(self.opacity), retain=True)

    def registerCallbacks(self):
        if self.pose_source:
            self.pose_topic = "/" + self.pose_source + "/" + self.name + "/pose" 
            # Subscribe to vehicle state update
            print("Subscribing to: {}".format(self.pose_topic))
            rospy.Subscriber(self.pose_topic, PoseStamped, self.pose_callback)


    def on_click_input(self, client, userdata, msg):
        print("Got click.")
        click_x, click_y, click_z, user = msg.payload.split(',')

        if str(msg.topic).find("mousedown") != -1:
            print("Got click: {} {} {}".format(click_x, click_y, click_z))
            pass
            

    def draw(self):
        # Fill mqtt message
        message = self.message.format(
            self.pos[0], self.pos[1], self.pos[2],
            self.quat[0], self.quat[1], self.quat[2], self.quat[3],
            self.scale[0], self.scale[1], self.scale[2], 
            self.color)

        # Draw object
        self.client.publish(self.base_topic, message, retain=True)

        message = self.text_message.format(
            self.pos[0] + self.offset[0], self.pos[1] + self.offset[1], self.pos[2] + self.offset[2],
            self.text_quat[0], self.text_quat[1], self.text_quat[2], self.text_quat[3],
            self.text_scale[0], self.text_scale[1], self.text_scale[2], 
            self.text)
        
        self.client.publish(self.text_topic, message, retain=True)


    def update(self):
        # Update pose
        self.draw()
        
        # Update animation
        if self.animate:
            tg_scene_string = self.base_topic + "/animation"
            cmd_string = "property: color; to: #FF0000; loop:true; dur: 500;"
            self.client.publish(tg_scene_string, cmd_string, retain=True)


    def remove(self):
        self.client.publish(self.base_topic, "", retain=True)
        self.client.publish(self.base_topic + "/click-listener", "", retain=True)
        self.client.publish(self.base_topic + "/animation", "", retain=True)
        self.client.publish(self.base_topic + "/material", "", retain=True)

        self.client.publish(self.text_topic, "", retain=True)
        self.client.publish(self.text_topic + "/click-listener", "", retain=True)
        self.client.publish(self.text_topic + "/animation", "", retain=True)
        self.client.publish(self.text_topic + "/material", "", retain=True)


    def pose_callback(self, pose_msg):
        self.pos = posFromPoseMsg(pose_msg)
        self.quat = quatFromPoseMsg(pose_msg)




###### DRONE ARENA CLASS ########
class DroneArenaClass(NodeArenaClass):
    def __init__(self, client, scene, name, id, shape="cube", color="#AAAAAA", animate=False,
        pos=[0,0.05,0], scale=[0.5, 0.5, 0.5], pose_source=None, on_click_clb=None):

        self.Active = False

        self.on_click = on_click_clb

        super(DroneArenaClass, self).__init__(client, scene, name, id, shape=shape, pos=pos, scale=scale, color=color, 
            animate=animate, pose_source=pose_source)

        self.registerServices()

        print("Created Arena Drone Object")

        
    def registerServices(self):
        #rospy.wait_for_service("/" + self.name + "/Commander_Node/goTo_srv")
        self.goTo = rospy.ServiceProxy("/" + self.name + "/Commander_Node/goTo_srv", GoTo)
        self.inTer = rospy.ServiceProxy("/" + self.name + "/gen_ImpTrajectoryAuto", GenImpTrajectoryAuto)
        self.land = rospy.ServiceProxy("/" + self.name + "/Commander_Node/land_srv", Land)

        pass


    def on_click_input(self, client, userdata, msg):
        click_x, click_y, click_z, user = msg.payload.split(',')
    
#        if (self.Active):
#            self.deactivate() 
#        else:
#            self.activate()
#
        if (self.on_click is not None):
            self.on_click(self.name) 
        else:
            print("Drone {}: Click Callback not specified\n".format(self.name))
 
    def deactivate(self):
        self.Active = False
        super(DroneArenaClass, self).set_color('#000022')
        self.color = "#000022"
        print("Drone {} Deselected\n".format(self.name))

    def activate(self):
        self.Active = True
        super(DroneArenaClass, self).set_color('#00FF00')
        print("Drone {} Selected\n".format(self.name))

    def isActive(self):
        return self.Active


###### TARGET ARENA CLASS #######
class TargetArenaClass(NodeArenaClass):
    def __init__(self, client, scene, name, id, shape="cube", color="#AAAAAA", animate=False,
        pos=[0,1,0], quat=[0,0,0,0], scale=[0.5, 0.5, 0.5], pose_source=None, on_click_clb=None):

        self.on_click = on_click_clb
        self.duration = 2

        self.marker_base_topic = scene + "/"  + "cube_{}00".format(id)
        #                                                    x  y  z  qx qy qz qw sx sy sz col
        self.marker_message = "cube_{}00".format(id)  + ",{},{},{},0,0,0,0,{},{},{},{},{}"

        self.last_time=None

        super(TargetArenaClass, self).__init__(client, scene, name, id, shape=shape, color=color, animate=animate, 
        pos=pos, quat=quat, scale=scale, pose_source=pose_source)

        self.draw_marker(status="on")

        #if self.transparent:
        self.client.publish(self.marker_base_topic + "/material", 
            "transparent: true; opacity: {}".format(self.opacity), retain=True)

        self.registerServices()

        print("Created Arena Target Object")

    def registerServices(self):
        pass
            
    def on_click_input(self, client, userdata, msg):
        click_x, click_y, click_z, user = msg.payload.split(',')

        # Create array with the target position

        print( "Target Position: " + str(click_x) + "," + str(click_y) + "," + str(click_z) )
        
        if (self.id == 6):
            self.draw_marker([click_x, str(float(click_y) + 0.7), click_z])
        else:
            self.draw_marker([click_x, click_y, click_z])

        tg_p = np.array([float(click_x), -float(click_z), float(click_y)])
        self.on_click(tg_p)

        self.last_time = rospy.get_time()

    def draw_marker(self, pos=[0,0,0], scale=[.1,.1,.1], color="#FF0000",status="on"):
        mqtt_string = self.marker_message.format(
            pos[0], pos[1], (pos[2]),
            scale[0], scale[1], scale[2],
            color,status)

        # Draw object
        self.client.publish(self.marker_base_topic, mqtt_string, retain=True)

    def update(self):
        # Update pose
        self.draw()
        # Clean up
        if self.last_time!=None and (rospy.get_time() - self.last_time) > self.duration:
            self.client.publish(self.marker_base_topic, self.marker_message.format(
            0,0,0,0,0,0,0,"off"), retain=True)

    def remove(self):
        self.client.publish(self.base_topic, "", retain=True)
        self.client.publish(self.base_topic + "/click-listener", "", retain=True)
        self.client.publish(self.base_topic + "/animation", "", retain=True)
        self.client.publish(self.base_topic + "/material", "", retain=True)

        self.client.publish(self.text_topic, "", retain=True)
        self.client.publish(self.text_topic + "/click-listener", "", retain=True)
        self.client.publish(self.text_topic + "/animation", "", retain=True)
        self.client.publish(self.text_topic + "/material", "", retain=True)

        self.client.publish(self.marker_base_topic, "", retain=True)
        self.client.publish(self.marker_base_topic + "/click-listener", "", retain=True)
        self.client.publish(self.marker_base_topic + "/animation", "", retain=True)
        self.client.publish(self.marker_base_topic + "/material", "", retain=True)




###### EDGE ARENA CLASS #######
class EdgeArenaClass(RArenaClass):
    def __init__(self, client, scene, name, id, start_node, end_node, color="#AAAAAA", 
        animate=False, ros_topic=None, interval=1000, packet_duration=200, active=True):

        self.last_time = rospy.get_time()
        self.visible = "on"
        self.packet_scale = [0.05,0.05,0.05]
        self.packet_duration = packet_duration
        self.ros_topic = ros_topic
        self.interval = interval
        self.active = active
        self.transparent=True
        self.opacity=0.5

        if start_node and end_node:
            self.start_node = start_node
            self.end_node = end_node
        else:
            rospy.loginfo("Error: Must supply start and end nodes to edge constructor")
        # Note: Should probably shut down here.
        super(EdgeArenaClass, self).__init__(client, scene, name, id, shape="line", color=color, animate=animate)

        print("Created Arena Edge Object")


    def initArenaObject(self):
        self.packet_base_topic = self.scene + "/"  + "cube_{}".format(self.id)
        #                                       x  y  z  x  y  z          col
        self.message = "line_{}".format(self.id) + ",{},{},{},{},{},{},0,0,0,0,{},{}"
        #                                                    x  y  z  qx qy qz qw sx sy sz col
        self.packet_message = "cube_{}".format(self.id)  + ",{},{},{},0,0,0,0,{},{},{},{},on"

        # Redraw objects
        self.remove()
        self.draw()

        # Enable click listener for object (allows it to be clickable)
        # self.client.publish(self.base_topic + "/click-listener", "enable", retain=True)
        # self.client.subscribe(self.base_topic + "/mousedown")
        # self.client.message_callback_add(self.base_topic + "/mousedown", self.on_click_input)


    def registerCallbacks(self):
        if self.ros_topic:
            self.ros_topic = "/" + self.ros_topic
            # Subscribe to vehicle state update
            print("Subscribing to: {}".format(self.ros_topic))
            rospy.Subscriber(self.ros_topic, PoseStamped, self.topic_callback)

    def topic_callback(self, msg):
        #If enough time has passed, trigger single packet animationg
        if rospy.get_time() > (self.last_time + self.interval/1000.0) and self.animate:
            tg_scene_string = self.scene + "/"  + "cube" + "_{}".format(self.id) + "/animation"
            cmd_string = "property: position; from: {} {} {}; to: {} {} {}; dur:{};".format(
                self.start_node.pos[0],self.start_node.pos[1],self.start_node.pos[2],
                self.end_node.pos[0],self.end_node.pos[1],self.end_node.pos[2],
                self.packet_duration)

            self.client.publish(tg_scene_string, cmd_string, retain=True)
            self.last_time = rospy.get_time()

        
    def draw(self):
        if self.active:
            color = self.color
            # self.visible = "on"
        else:
            color = "#000000"
            # self.visible = "off"


        mqtt_string = self.message.format(
            self.start_node.pos[0], self.start_node.pos[1], self.start_node.pos[2],
            self.end_node.pos[0], self.end_node.pos[1], self.end_node.pos[2],
            color, self.visible)

        # Draw object
        self.client.publish(self.base_topic, mqtt_string, retain=True)

        # Fill packet message
        packet_string = self.packet_message.format(
            self.start_node.pos[0], self.start_node.pos[1], self.start_node.pos[2], 
            self.packet_scale[0], self.packet_scale[1], self.packet_scale[2], self.color)
        
        self.client.publish(self.packet_base_topic, packet_string, retain=True)

        if self.transparent:
            self.client.publish(self.packet_base_topic + "/material", 
            "transparent: true; opacity: {}".format(self.opacity), retain=True)
            

    def update(self):
        # Update pose
        self.draw()


    def remove(self):
        self.client.publish(self.base_topic, "", retain=True)
        self.client.publish(self.base_topic + "/click-listener", "", retain=True)
        self.client.publish(self.base_topic + "/animation", "", retain=True)
        self.client.publish(self.base_topic + "/material", "", retain=True)

        self.client.publish(self.packet_base_topic, "", retain=True)
        self.client.publish(self.packet_base_topic + "/click-listener", "", retain=True)
        self.client.publish(self.packet_base_topic + "/animation", "", retain=True)
        self.client.publish(self.packet_base_topic + "/material", "", retain=True)


    def start_animation(self):  
        self.animate = True

    
    def stop_animation(self):
        self.animate = False
        self.client.publish(self.base_topic + "/animation", "", retain=True)
        self.client.publish(self.packet_base_topic + "/animation", "", retain=True)


    def activate(self):
        self.active = True


    def deactivate(self):
        self.active = False


###### SETPOINT ARENA CLASS ########
class SetpointArenaClass(NodeArenaClass):
    def __init__(self, client, scene, name, id, shape="cube", color="#AAAAAA", animate=False,
        pos=[0,0.05,0], scale=[0.5, 0.5, 0.5], pose_source=None, on_click_clb=None):

        self.Active = False

        self.on_click = on_click_clb

        super(DroneArenaClass, self).__init__(client, scene, name, id, shape=shape, pos=pos, scale=scale, color=color, 
            animate=animate, pose_source=pose_source)

        self.registerServices()

        print("Created Arena Drone Object")

        
    def registerServices(self):
        #rospy.wait_for_service("/" + self.name + "/Commander_Node/goTo_srv")
        self.goTo = rospy.ServiceProxy("/" + self.name + "/Commander_Node/goTo_srv", GoTo)
        self.inTer = rospy.ServiceProxy("/" + self.name + "/gen_ImpTrajectoryAuto", GenImpTrajectoryAuto)
        self.land = rospy.ServiceProxy("/" + self.name + "/Commander_Node/land_srv", Land)

        pass


    def on_click_input(self, client, userdata, msg):
        click_x, click_y, click_z, user = msg.payload.split(',')
    
#        if (self.Active):
#            self.deactivate() 
#        else:
#            self.activate()
#
        if (self.on_click is not None):
            self.on_click(self.name) 
        else:
            print("Drone {}: Click Callback not specified\n".format(self.name))
 
    def deactivate(self):
        self.Active = False
        super(DroneArenaClass, self).set_color('#000022')
        self.color = "#000022"
        print("Drone {} Deselected\n".format(self.name))

    def activate(self):
        self.Active = True
        super(DroneArenaClass, self).set_color('#00FF00')
        print("Drone {} Selected\n".format(self.name))

    def isActive(self):
        return self.Active