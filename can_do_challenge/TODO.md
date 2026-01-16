# TODO

- Review the gripper code on the Teensy boards to ensure stepping actions never return `RUNNING` while stepping.
- Review places where `Sequence` was used instead of a guarded `ReactiveSequence` to preserve safety subtree responsiveness.
