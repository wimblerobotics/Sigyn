# Human Interaction Specification

**Document status:** Draft v0.1  
**Last updated:** 2026-03-03  
**Scope:** Human notification, alert escalation, override authorization, and the
people/contact model for the Sigyn robot platform  
**See also:**
- `docs/specs/wr_ros_teensy.spec.md` — the package that publishes HumanAlert events
- `docs/specs/safety_system.spec.md` — authority chain and fault model
- `docs/specs/sigyn.spec.md` — overall system specification

---

## Table of Contents

1. [Purpose and Motivation](#1-purpose-and-motivation)
2. [People Model](#2-people-model)
3. [Alert Tiers and Channels](#3-alert-tiers-and-channels)
4. [Fault-to-Alert Mapping](#4-fault-to-alert-mapping)
5. [Human Override and Fault Clearing](#5-human-override-and-fault-clearing)
6. [Notification Delivery Infrastructure](#6-notification-delivery-infrastructure)
7. [Privacy and Security](#7-privacy-and-security)
8. [ROS 2 Interface Summary](#8-ros-2-interface-summary)
9. [Testing Requirements](#9-testing-requirements)
10. [Open Questions](#10-open-questions)

---

## 1. Purpose and Motivation

The Sigyn robot will eventually perform life-critical tasks for a person who may
have limited mobility or be alone. When the robot encounters a condition it cannot
resolve autonomously — from a wheel-entangling cord to a battery failure — a human
must be informed, must be able to understand the situation, and in some cases must
be able to authorize the robot to continue.

The human interaction layer has three responsibilities:

1. **Notification:** Reliably deliver the right information to the right people
   through channels appropriate to the urgency of the situation.

2. **Override / clearance:** Allow a human to confirm that an underlying physical
   condition has been resolved and that the robot may proceed.

3. **Transparency:** All human interactions are logged with full context —
   who was notified, what was said, who responded, and what they authorized.

**Scope of this document:** Everything on the PC (ROS 2) side. No human interaction
data flows to the Teensy side except for a person nickname string used in audit log
entries on the SD card.

---

## 2. People Model

### 2.1 People table

The people table is a JSON file (`config/people.json`) that associates a short
nickname with contact information. This file is **never committed to git** — see
Section 7.

```json
{
  "people": [
    {
      "nickname": "wim",
      "display_name": "William Rotenberg",
      "email": "user@example.com",
      "phone_e164": "+15555550100",
      "preferred_method": "email",
      "notify_tiers": [1, 2, 3, 4]
    }
  ]
}
```

Fields:

| Field | Type | Description |
|---|---|---|
| `nickname` | string (≤6 chars) | Short, unique identifier used in logs, fault events, and Teensy SD log entries |
| `display_name` | string | Full name, used in email body / SMS body |
| `email` | string | Email address for tier 3+ notifications |
| `phone_e164` | string | E.164 phone number for SMS (tier 2+) |
| `preferred_method` | string | `"email"`, `"sms"`, `"both"`, `"log_only"` |
| `notify_tiers` | int[] | List of tiers (1–4) this person should receive |

### 2.2 Per-person capabilities (future)

In this initial design, all people in the table are equally capable of responding
to any fault. There is no role-based access control — any person who knows the
correct reason string can clear any clearable fault.

A future revision may add a `clearable_fault_classes` field per person and an
`authorization_token` mechanism for an app-based interface. This is noted as
OQ-HI-1.

### 2.3 Nickname usage

The nickname is the identifier that travels in override requests, log entries,
and the Teensy SD log. Using a short nickname (≤6 characters) keeps log lines
concise and ensures the nickname is useful in limited-space contexts (e.g., a
Teensy SD log entry that logs a fault clearance with the clearing identity).

Nicknames must be lowercase alphanumeric, no spaces, no special characters.
They are case-insensitively unique within the table.

### 2.4 People table versioning

The people table is loaded fresh at node startup. Changes to `people.json` require
a node restart to take effect. A future feature may use file-watch to reload without
restart; for now, restart is required.

---

## 3. Alert Tiers and Channels

### 3.1 Tier definitions

| Tier | Name | Severity mapping | Description |
|---|---|---|---|
| 1 | EMERGENCY | SYSTEM_SHUTDOWN | Contact all people by all available methods simultaneously; no delay |
| 2 | URGENT | EMERGENCY_STOP | Push notification + audible alert on the main PC (tone or voice); phone/SMS if no acknowledgment within `tier2_escalate_s` (default: 120 s) |
| 3 | ADVISORY | DEGRADED | App push notification and/or email; no audible |
| 4 | INFORMATIONAL | WARNING | Appended to a periodic status report; no immediate delivery |

Tier assignment may be overridden on a per-fault-type basis in configuration
(see Section 4.2). The default mapping above is used when no fault-specific override
exists.

### 3.2 Delivery channels

| Channel | Trigger tier | Mechanism | Requires |
|---|---|---|---|
| Audible tone | ≤ 2 | Play audio file via PC speakers | `aplay` / system audio |
| Audible voice | ≤ 2 (optional) | TTS via `espeak-ng` | `espeak-ng` installed |
| App push notification | ≤ 3 | Publish to `/sigyn/safety/human_alert` topic; downstream app node handles delivery | App node running |
| Email | ≤ 3 | SMTP via config (see Section 6.2) | SMTP credentials file |
| SMS | ≤ 2 | Gateway API (Twilio or similar) | API credentials file |
| Periodic status report | = 4 | Appended to `~/.ros/sigyn_status.log`; optionally emailed on a schedule | Email config |

The notification delivery node subscribes to `/sigyn/safety/human_alert` and
handles tier routing. That node is not part of `wr_ros_teensy`; it is a separate
package. `wr_ros_teensy` only publishes the `HumanAlert` event.

### 3.3 Which people get notified

**Default:** All people in the people table whose `notify_tiers` list includes the
event's tier are notified.

**Per-fault override:** Some fault types specify an explicit `notify_people` list —
a subset of nicknames. When present, only those people are notified regardless of
their `notify_tiers` preference. Example use case: a minor sensor calibration fault
that only the primary operator needs to know about.

For tier 1 (EMERGENCY), all people in the table are always notified, regardless of
any per-fault configuration. The "robot is on fire" case requires everyone to pay
attention, and it would be a design error for a configuration mistake to suppress
those alerts.

---

## 4. Fault-to-Alert Mapping

### 4.1 Alert trigger conditions

A `HumanAlert` event is generated when any of the following occur:

- A fault with `requires_human_ack = true` is activated.
- Any fault reaches `EMERGENCY_STOP` severity.
- Any `latching` fault activates (requires explicit clearance to deactivate).
- A fault remains active for longer than its `alert_escalation_timeout_s` without
  being acknowledged or cleared (the same fault re-generates a higher-tier alert).

### 4.2 Per-fault notification configuration

Fault notification behavior is configurable per fault type in
`config/fault_notifications.json`. Fields:

```json
{
  "fault_notifications": [
    {
      "fault_class": "VL53L0X",
      "tier_override": null,
      "notify_people": null,
      "clearable_by_human": true,
      "permitted_clear_reasons": ["obstacle_removed", "testing"],
      "escalation_timeout_s": 300
    },
    {
      "fault_class": "SPIN_IN_PLACE",
      "tier_override": 2,
      "notify_people": ["wim"],
      "clearable_by_human": true,
      "permitted_clear_reasons": ["cord_cleared", "area_cleared", "testing"],
      "escalation_timeout_s": 60
    },
    {
      "fault_class": "BATTERY_CRITICAL",
      "tier_override": 2,
      "notify_people": null,
      "clearable_by_human": false,
      "permitted_clear_reasons": [],
      "escalation_timeout_s": 60
    }
  ]
}
```

`tier_override: null` means use the default severity-to-tier mapping. `notify_people: null`
means use the default people-from-tier logic.

### 4.3 Override authorization table

Each fault class with `clearable_by_human: true` must have a non-empty
`permitted_clear_reasons` list. When a human submits an override or clear request,
the provided reason must match one of the permitted reasons (case-insensitive). If
it does not, the service returns the list of valid reasons.

This table is the authoritative definition of what "human intervention" means for
each fault type. It must be version-controlled (it does not contain personal data)
and is part of the `wr_ros_teensy` package configuration.

---

## 5. Human Override and Fault Clearing

### 5.1 Overview

Two distinct operations require human involvement:

1. **Override:** The underlying fault condition may still be present, but a human
   asserts that it is safe to continue anyway. The fault remains in the registry
   (visible in state reports) but does not contribute to the aggregated severity.
   Override expires at the next detection cycle if the condition persists.

2. **Clear:** The underlying physical condition has been resolved. The human certifies
   this. The safety coordinator removes the fault reason from the active reasons set
   and, if all reasons are cleared, deactivates the fault entirely and sends the
   board a `FAULT_CLEAR_CMD`.

### 5.2 Interface

Both operations come through ROS 2 services defined in `wr_ros_teensy.spec.md`
Section 8.3:

- `/sigyn/safety/human_override` — override without clearing
- `/sigyn/safety/request_clear` — assert condition is resolved and request clear

Both require:
- `person_id` — nickname from the people table
- `reason` — from the permitted reasons list for the fault class (clear) or any
  descriptive reason string (override)
- `fault_id` — the specific fault to act upon
- `board_id` and `instance` — to identify the exact fault entry

The `notification_method` field on `human_override` allows the responding human to
specify how they want to receive confirmation: `"email"`, `"sms"`, or `"silent"`.

### 5.3 Validation

Before applying an override or clear:
1. Verify `person_id` is in the people table.
2. Verify `reason` is in the `permitted_clear_reasons` list for this fault class
   (for clear requests). Override requests accept any non-empty reason string.
3. Verify the fault entry exists and is currently active.
4. For clear requests, verify `clearable_by_human` is `true` for this fault class.

If validation fails, the service returns `success: false` with a descriptive
`message` field. If the reason was wrong, `permitted_reasons` is populated in the
response.

### 5.4 Audit log entry format

Every override and clear is written to the safety event log (`sigyn_safety_events.log`)
as a JSON record:

```json
{
  "event": "HUMAN_OVERRIDE",
  "timestamp": "2026-03-03T10:22:15.123Z",
  "fault_id": "VL53L0X_RING3",
  "board_id": 1,
  "instance": 4,
  "person": "wim",
  "reason": "obstacle_removed",
  "bt_action": "NavigateToPose",
  "active_faults_at_time": 2
}
```

The `bt_action` field records the currently active behavior tree action at the
time of the override, providing forensic context. If no BT action has been received
recently, an empty string is used.

### 5.5 Fault clear propagation to Teensy

When a `request_clear` is accepted:
1. The `clear_reason` and `cleared_by` (person nickname) are written to the PC log.
2. A `FAULT_CLEAR_CMD` message is sent to the Teensy board: `FAULT_CLEAR_CMD<N>:{"fid":"VL53L0X_RING3","inst":4,"by":"wim","why":"obstacle_removed"}`.
3. The board logs the clearance event to its SD card with the nickname.
4. The board confirms via a `FAULT_CLEAR` response message.
5. The PC registry updates `active_reasons` accordingly.
6. The PC does NOT wait for e-stop to be signaled cleared before responding to
   the service call; it waits only for the board's fault-level acknowledgment.

---

## 6. Notification Delivery Infrastructure

### 6.1 Notification node

`wr_ros_teensy` publishes `HumanAlert` events; a separate `sigyn_notifier` node
(not yet implemented) subscribes and handles delivery. The interface between them
is the `/sigyn/safety/human_alert` topic, typed as `wr_interfaces/HumanAlert`.

The notifier node is responsible for:
- Routing alerts to the correct people and channels based on tier and per-fault config.
- Managing escalation timers (re-notify if not acknowledged).
- Tracking acknowledgment state.
- Reading SMTP and SMS gateway credentials from credential files (excluded from git).

### 6.2 Credential management

Credentials for email (SMTP password or app password) and SMS gateway (API key)
must NEVER appear in any file that could be committed to git. They are stored in:

- `config/credentials.json` — **excluded from git** by `.gitignore`

A version-controlled example (`config/credentials.example.json`) ships with placeholder
values and setup instructions.

### 6.3 Audible alerts

Tier 1 and 2 alerts trigger an audible alert on the main PC. Implementation:
- A short audio file is played via `aplay` (configurable in `config/alerts.json`).
- If `espeak-ng` is installed, a spoken message may also be generated.
- The audible alert plays immediately on the PC running `wr_ros_teensy`; there is
  no network delivery for audible alerts.

---

## 7. Privacy and Security

### 7.1 Data that must never reach the Teensy side

- Full names, email addresses, phone numbers.
- Authentication tokens, passwords, API keys.
- Any personally identifiable contact information.

The only human-related data sent to the Teensy is a person's **nickname** string
(e.g., `"wim"`) appearing in fault clearance log entries. The nickname is a short,
non-identifying token useful only for correlating PC-side and Teensy-side logs.

### 7.2 Files excluded from git

The following files must appear in `.gitignore`:

```
config/people.json
config/credentials.json
```

The repository ships template files:
```
config/people.example.json
config/credentials.example.json
```

Each template includes a comment at the top: "Copy this file to `people.json` (or
`credentials.json`), fill in real values, and never commit the result."

### 7.3 Logging retention

The safety event log and notification log may contain nicknames and action context.
They do not contain contact details (those are only in the credential files and
never logged). Log files should be treated as internal operational data.

---

## 8. ROS 2 Interface Summary

Published by `wr_ros_teensy`:

| Topic / Service | Direction | Type | Description |
|---|---|---|---|
| `/sigyn/safety/human_alert` | Published | `wr_interfaces/HumanAlert` | Alert event for notifier node |
| `/sigyn/safety/human_override` | Service (server) | `wr_interfaces/srv/HumanOverride` | Accept override request |
| `/sigyn/safety/request_clear` | Service (server) | `wr_interfaces/srv/RequestFaultClear` | Accept clear request |
| `/sigyn/safety/fault_enumerate` | Service (server) | `wr_interfaces/srv/FaultEnumerate` | Full fault registry dump |

Published by `sigyn_notifier` (future package, not yet implemented):

| Topic | Direction | Type | Description |
|---|---|---|---|
| `/sigyn/safety/alert_ack` | Published | `wr_interfaces/AlertAck` | Human acknowledgment of an alert |

---

## 9. Testing Requirements

### 9.1 Unit tests (no hardware, no ROS 2)

- People table loader: valid file; missing required fields; duplicate nicknames;
  empty file; missing file (returns empty table, logs warning).
- Alert tier calculation: fault severity maps to correct tier; per-fault override
  applied; EMERGENCY_STOP always triggers all people.
- Override validation: valid person + valid reason → success; invalid person → fail;
  invalid reason → fail + returns permitted list; non-clearable fault → fail.

### 9.2 Integration tests

- HumanAlert event published when fault reaches EMERGENCY_STOP severity.
- `human_override` service: valid request applied; fault entry updated; audit log
  entry written.
- `request_clear` service: valid request sends `FAULT_CLEAR_CMD` to mock serial
  bridge; board response causes `active_reasons` removal; e-stop cleared only when
  last reason removed.
- Tier 1 event: all people in table appear in `notify_people` of published alert.

---

## 10. Open Questions

| ID | Question | Affects |
|---|---|---|
| OQ-HI-1 | Should some faults be clearable only by specific designated people? E.g., only a trained technician can clear a hardware fault. Requires role-based authorization in the people table. | Section 2.2, 5.3 |
| OQ-HI-2 | Should there be a physical "clear" button on the robot chassis? If so, how is it integrated — does pressing it trigger the `request_clear` service with a special `cleared_by="physical_button"` value? | Section 5.2 |
| OQ-HI-3 | What is the escalation policy for un-acknowledged alerts? After `tier2_escalate_s` with no acknowledgment, escalate to tier 1? Send again? This needs a defined state machine. | Section 3.1 |
| OQ-HI-4 | Is there a web UI or app for human override submission? If so, that UI must call `/sigyn/safety/human_override` via ROS 2 bridge (rosbridge or rclnodejs). Out of scope for initial implementation. | Section 5.2 |
| OQ-HI-5 | Should the notifier node support mobile push notifications (e.g., via Firebase Cloud Messaging)? This would require an internet connection and credentials management beyond email/SMS. | Section 6.1 |
