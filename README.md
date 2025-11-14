# UDP Client ROS 2 Package

A lightweight, open-source ROS 2 package that sends structured UDP messages based on incoming ROS 2 commands.  
This package demonstrates how to bridge ROS 2 topics to raw UDP communication in a clean, extensible way.

---

## 🚀 Features

- Subscribes to a ROS 2 topic (`cmd/input`)
- Packs command values into a binary UDP message
- Sends UDP packets to a configurable IP and port
- Fully open-source safe (no proprietary message types)
- Easy to extend for real robotic or networking applications

---

# ros2_udp_bridge

A ROS 2 package that sends `ActorCmd` messages over UDP to an external server.

## Features
- Listens to `act01/actor_cmd` topic.
- Packs commands (`acc`, `kappa`) into a binary UDP payload.
- Sends UDP packets to a configurable IP/port.
- Designed for simulation, robotics, and control systems.

## Run

ros2 run ros2_udp_client_for_robotic_adas_targets udp_clientnode

source install/setup.bash

ros2 run udp_client.py

