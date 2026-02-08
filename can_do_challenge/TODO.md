# TODO

- Review the gripper code on the Teensy boards to ensure stepping actions never return `RUNNING` while stepping.
- Review places where `Sequence` was used instead of a guarded `ReactiveSequence` to preserve safety subtree responsiveness.
- Update `ExtendTowardsCan` to continuously monitor alignment during extension:
  1. Verify gripper Z-height remains correct relative to the can.
  2. Verify robot rotation keeps the can's vertical centerline aligned with the `parallel_gripper_base_plate` centerline.
  These continuous checks will guarantee the gripper reaches the correct grasp pose.
- Verify Grasp Success After Retraction:
  - Add logic to check if the can is still held after `RetractExtender`.
  - Design recovery logic (e.g., retry grasp, re-detect) if the grasp fails.
  - Note: The extension logic targets placing the front of the can touching (or almost touching) the `parallel_gripper_base_plate`.

- bt_nodes sim vs real parity:
  - Align `WaitForNewOAKDFrame` behavior (sim currently uses fixed delay; real waits on heartbeat timestamps).
  - Align node type semantics for `MoveTowardsCan` (sim uses `StatefulActionNode` while real uses `SyncActionNode`).
  - Add missing `OAKDDetectCan` node to sim or stub it to mirror real behavior.
  - Unify `initializeObjectDetection` subscriptions and message types:
    - OAK-D topics (/oakd_top vs /oakd) and heartbeat mechanism (processed header vs Detection2D heartbeat).
    - Pi camera topics (/gripper/can_detection vs /gripper/camera/detections) and message types.
  - Make `getFreshPiDetection` behavior consistent (sim uses blocking wait; real is non-blocking with different throttling).
  - Make `piDetectionInRange` consistent (real accepts 2D Z=0/0.30 defaults; sim uses strict range).
  - Align `ObjectDetectionState` defaults/fields (e.g., `WITHIN_REACH_DISTANCE`, 2D centers, class/score tracking).
  - Perform a full parity review of all nodes in `bt_nodes.cpp` vs `bt_nodes_real.cpp` and list/resolve remaining mismatches.
