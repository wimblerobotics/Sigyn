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
