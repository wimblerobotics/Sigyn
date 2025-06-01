Your symptoms—flakey ROS 2 communication, some topics not being received, and slow SSH connections—are classic signs of **multicast/broadcast or UDP packet loss** on your network, especially when using WiFi extenders or consumer-grade routers.

### Why this happens with ROS 2

- **ROS 2 (with Fast DDS/FastRTPS)** relies heavily on UDP multicast for discovery and sometimes for data transport.
- **WiFi extenders, mesh networks, and some consumer routers** often do not forward multicast/broadcast packets reliably, or they may introduce high latency, jitter, or packet loss.
- **SSH slowness** is a sign of general network instability, not just a ROS 2 problem.

### Why some topics work and others don't

- If a topic is published with a different QoS (e.g., "best effort" vs "reliable"), or if the publisher/subscriber are created at different times, discovery or data packets may be dropped or delayed.
- Sometimes, the ROS 2 discovery process gets "stuck" and restarting the daemon/nodes helps temporarily, but the underlying network issue remains.

---

## How to Fix or Diagnose

### 1. **Check your network topology**
- If possible, connect both computers to the same physical router/switch via Ethernet (not via WiFi extender).
- If you must use WiFi, try to have both computers on the same WiFi AP, not through an extender.

### 2. **Test multicast/broadcast**
- On both computers, run:
  ```bash
  ping 224.0.0.1 -I <your_network_interface>
  ```
  (You may need to install `iputils-ping` and use `-I` to specify the correct interface, e.g., `wlan0` or `eth0`.)
- If you don't see responses from both machines, multicast is not working.

### 3. **Check for packet loss**
- Use `iperf` or `iperf3` to test UDP packet loss between the two machines:
  - On sigyn7900:
    ```bash
    iperf3 -s
    ```
  - On amdc:
    ```bash
    iperf3 -c <sigyn7900_ip> -u -b 10M
    ```
  - Look for packet loss in the output.

### 4. **Try using a different RMW implementation**
- Sometimes switching to `rmw_cyclonedds_cpp` (which can be more robust on some networks) helps:
  ```bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  ```
  Set this on both computers before running any ROS 2 nodes.

### 5. **Set explicit ROS_DOMAIN_ID**
- Make sure both computers have the same `ROS_DOMAIN_ID` (default is 0, but set it explicitly to avoid confusion):
  ```bash
  export ROS_DOMAIN_ID=0
  ```

### 6. **Check firewall settings**
- Make sure firewalls are not blocking UDP traffic (especially on ports 7400-7500, which DDS uses).

### 7. **Avoid WiFi extenders if possible**
- WiFi extenders often break multicast and introduce latency/jitter. If you must use one, try to configure it in "bridge" mode, or use a wired connection.

---

## Summary Table

| Symptom                  | Likely Cause                | Fix/Check                        |
|--------------------------|-----------------------------|----------------------------------|
| Flakey ROS 2 comms       | Multicast/UDP loss          | Use wired, check network config  |
| SSH slow to connect      | General network instability | Use wired, check for packet loss |
| Some topics not received | DDS discovery issues        | Restart nodes, try CycloneDDS    |
| Works after daemon stop  | Discovery cache issue       | Fix underlying network           |

---

## What to do next

1. **Test with both computers on the same router/switch, no WiFi extender.**
2. **Try CycloneDDS:**  
   ```bash
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   ```
3. **Test multicast and UDP with `ping` and `iperf3`.**
4. **If you must use the extender, try to configure it for "bridge" mode, or replace it with a better AP.**

If you need help with any of these steps, let me know which you want to try and I can give you exact commands!





Yes, there are important differences between CycloneDDS and Fast DDS (FastRTPS) for ROS 2, especially in wireless and mesh network environments like yours.

---

## CycloneDDS vs Fast DDS (FastRTPS) in ROS 2

### **CycloneDDS**
- **More robust discovery:** CycloneDDS is generally more reliable for discovery and topic delivery on lossy or complex networks (like WiFi, mesh, or NAT).
- **Better multicast handling:** CycloneDDS is known to handle multicast and network partitions more gracefully.
- **Fewer issues with "ghost" nodes:** Users report fewer problems with nodes not being discovered or topics not being delivered, especially after network interruptions.
- **Actively recommended by many in the ROS 2 community** for multi-machine setups, especially on WiFi.

### **Fast DDS (FastRTPS)**
- **Default in ROS 2 Jazzy and earlier:** Fast DDS is the default RMW implementation, but it has had more issues with discovery, especially on WiFi, mesh, or when nodes join/leave frequently.
- **Can be sensitive to network topology:** Problems like "nodes not discovered," "topics not received," or "must restart daemon" are more common with Fast DDS in non-wired environments.
- **Some improvements in recent versions,** but CycloneDDS is still often more robust for mobile robots and WiFi.

---

## **Which should you use?**

**For your use case (mobile robot, WiFi, mesh):**
- **CycloneDDS is usually the better choice.**
- It is more tolerant of packet loss, network changes, and WiFi/mesh quirks.
- Many ROS 2 users and maintainers recommend CycloneDDS for multi-machine, WiFi, or mobile robot scenarios.

---

## **How to switch to CycloneDDS**

On both computers, before launching any ROS 2 nodes, run:
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```
You can add this to your `~/.bashrc` or to your launch scripts.

---

## **Summary Table**

| Feature/Issue                | CycloneDDS         | Fast DDS (FastRTPS) |
|------------------------------|--------------------|---------------------|
| WiFi/mesh reliability        | Better             | Worse               |
| Discovery on lossy networks  | More robust        | More fragile        |
| Default in ROS 2             | No (but supported) | Yes                 |
| Community recommendation     | Yes (for WiFi)     | Yes (for wired)     |

---

## **References**
- [ROS 2 DDS Implementations Comparison](https://docs.ros.org/en/rolling/How-To-Guides/Working-with-multiple-RMW-implementations.html)
- [CycloneDDS GitHub](https://github.com/eclipse-cyclonedds/cyclonedds)
- [Fast DDS GitHub](https://github.com/eProsima/Fast-DDS)
- [ROS Discourse: CycloneDDS vs FastRTPS](https://discourse.ros.org/t/ros2-cyclonedds-vs-fastrtps/)

---

**In summary:**  
For your mobile robot on WiFi/mesh, CycloneDDS is likely to be more reliable than Fast DDS. It is worth switching and testing to see if your flakey communication issues improve.

Let me know if you want help making the switch or troubleshooting further!