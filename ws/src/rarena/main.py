#!/bin/python

## IMPORTS
import sys
from signal import signal, SIGINT
from sys import exit
from arena import *
import numpy as np
import json
import os

import rospy
import rospkg

from classes import ROSArenaObject
from classes import PawnObject 


import time

## GLOBAL
host = "arena.andrew.cmu.edu"
realm = "realm"

# Arena Main object 
# The scene should be specified with the environmental variable SCENE
arena_scene = Scene(host=host, realm=realm, scene=os.environ['SCENE'])
# List of arena entities
arena_entities = dict()


def handler(signal_received, frame):
    global arena_scene
    global arena_entities

    # Handle any cleanup here
    print('SIGINT or CTRL-C detected. Exiting gracefully')

    for (key, value) in arena_entities.items():
        value.delete_obj()

    time.sleep(5)

    print('TERMINATED')

    exit(0)


def load_entities():
    global arena_entities
    global arean_scene
    rospack = rospkg.RosPack()
    stream = open(rospack.get_path('rarena') + '/config/atlas.json', 'r')

    # Load information from the configuration file
    json_data = json.load(stream)
    print("Loading Objects...\n")
    for el in json_data['entities']:
        pos = el["pos0"];
        entity = None
        if (el["type"] == "object"):
            print("Found {} with name {} @ {}\n".format(
                el['type'], el['name'], el['pos0']))
            entity = ROSArenaObject(
                    arena_srv = arena_scene,
                    object_id = el["name"],
                    obj_type = el["shape"],
                    scale = el["scale"],
                    color = el["color"],
                    opacity = el["opacity"],
                    position = Position(pos[0], pos[1], pos[2]))

        if (el["type"] == "pawn"):
            print("Found {} with name {} @ {}\n".format(
                el['type'], el['name'], el['pos0']))
            entity = PawnObject(
                    arena_srv = arena_scene,
                    object_id = el["name"],
                    obj_type = el["shape"],
                    scale = el["scale"],
                    color = el["color"],
                    opacity = el["opacity"],
                    position = Position(pos[0], pos[1], pos[2]))

        if (entity is not None):
            arena_entities[el["name"]] = entity 
    print("Loading Objects: Done!\n")


@arena_scene.run_once
def setup():
    signal(SIGINT, handler)

    # Load Objects from file.
    load_entities()


@arena_scene.run_forever(interval_ms=100)
def periodic():
    # Update the entities of the arena
    for key, entity in arena_entities.items():
        # Update the Object
        entity.arena_update()
    

if __name__ == '__main__':
    rospy.init_node('RArena_node')
    # rospy.on_shutdown(remove_objects)

    # Start Arena tasks
    arena_scene.run_tasks()

    rospy.spin()
    
    


