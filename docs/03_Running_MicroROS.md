# Running MicroROS

This allows you to start up an agent to communicate over ROS.

Terms:

* Agent: The application on *your* computer bridging the microcontroller to the ROS network
* Client: The application on the microcontroller handling ROS objects

## CAN Bus Interface Set Up

To connect over CAN bus, you must connect the CAN bus adapter into the bus. This is typically done over the Orin CAN
link on the Camera Cage Breakout Board. Unplug the cable running to the Orin, and connect the USB to CAN bus adapter
to the Camera Cage BB instead. You can then plug the USB to CAN adapter into your computer. For smart battery housings,
this cable is connected into the charge cable.

Ensure that you see a `can0` interface appear in your network interface list by running:

    ip addr

You will now need to set the proper baud rate. Refer to the robot definition header file in
`lib/titan_boards/include/robots` for the most up to date baud rates. As of writing this document, the internal baud
rate is 1 Mbps (1000000) and the external CAN bus is ran at 250 kbps (250000). For example, to configure the adapter
for the internal CAN bus, run:

    ip link set can0 up qlen 1000 type can bitrate 1000000

If a microcontroller is powered up and configured for CAN bus, running this command should display traffic (it might
take up to 10 seconds if the microcontroller is the only node on the network):

    candump can0

## Ethernet Interface Set Up

The MicroROS ethernet transport requires that the agent (your computer), have a fixed, known IP address. This is set
to the IP address of the robot computer (configured in the robot definition header file). If you a client to
connect to your computer, you must set your IP address to the robot's static IP. **Ensure you are not connected to the
same network as the primary vehicle computer, or else you will run into issues!** This can be done via the network
manager gui on in the Ubuntu Desktop settings.

## Starting the Agent

Ensure that you have ROS installed on your computer, and you built the micro ros agent.

To connect over can bus, run the following command:

    ros2 run micro_ros_agent micro_ros_agent can --dev can0

To connect over ethernet, run the following command:

    ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

## Basic ROS communication

To view a topic being published from the client, you can run

    ros2 topic echo /robot_name/topic/name/here

To publish a topic once, you can run the command: (remove the `-1` to publish continuously)

    ros2 topic pub -1 /robot_name/topic/name/here msg_type_pkg/msg/msg_type_name "{}"

Use tab complete to help you with the message type and message contents (the `{}`). Note that `{}` is for empty message
types. For message types with data, **first add quotes**, then press tab until a hint for what the format is displayed.
After putting the first couple of letters of the format, it should auto-complete to the full structure. You can now fill
out the message.
