#ifndef TITAN__CANMORE__ETHERNET_DEFS_H_
#define TITAN__CANMORE__ETHERNET_DEFS_H_

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Ethernet-specific definitions for CANmore protocol.
 * This enables RP2040s which use an Ethernet interface to communicate with the same protocols as CANmore, specifically
 * for accessing the CANmore control interface.
 *
 * Note that the ethernet protocol inherits from the data fields defined by the CANmore protocol. However, Ethernet has
 * its own mechanism to allocate "channels" (known as ports) which will be defined in this file, now mapping canmore
 * services to ports rather than channels.
 *
 * Although CANmore was designed with CAN traffic in mind (max MTU of 8 bytes), transactions can still occur over UDP
 * sockets, as they have similar characteristics of packet-oriented best-effort protocols (alhtough the error-detction
 * of UDP is not as good as CAN).
 *
 * This file defines how the CAN-oriented channel/ID gets mapped to the UDP-oriented IP/ports. UDP traffic which is
 * configured to be broadcast (discovery-based) will be sent over a broadcast IP defined in this file. Traffic defined
 * to be agent-to-client/client-to-agent, will require an agent request for every client response. For example, the
 * CANmore heartbeat is sent over a broadcast packet to allow for discovery over the network. The register mapped
 * protocol also works well with UDP, as each request from the register mapped client (the agent on the CAN
 * netowrk/remote device over UDP) must sent the request, and will receive one response from the register mapped server
 * (the client on the CAN network).
 *
 * To avoid collisions with ROS2 DDS port allocations, all ports should remain below port 7400. To allow for binding to
 * interfaces without superuser on POSIX systems, the broadcast port should be above 1023 in the Registered Port Range.
 */

#define CANMORE_TITAN_ETH_BROADCAST_IP                                                                                 \
    { 255, 255, 255, 255 }
#define CANMORE_TITAN_ETH_HEARTBEAT_BROADCAST_PORT 2201
#define CANMORE_TITAN_ETH_CONTROL_INTERFACE_PORT 2202

#ifdef __cplusplus
}
#endif

#endif
