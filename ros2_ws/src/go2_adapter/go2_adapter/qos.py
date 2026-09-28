#!/usr/bin/env python3
"""QoS profiles for talking to the Go2. Encodes MEASURED DDS facts, not defaults.

Source: inventory-go2_ros2.md section 2, from logs/go2/s2_topic_info_v_key.txt --
19 endpoints across 11 topics, every one identical:

    every robot PUBLISHER   RELIABLE, KEEP_LAST(1), VOLATILE, infinite lease
    every robot SUBSCRIBER  BEST_EFFORT, KEEP_LAST(1), VOLATILE

WARNING ON SCOPE: 11 of 119 topics were sampled. The uniformity is striking and
the remaining 108 are unsampled, so "no exceptions" is a generalisation from a
sample, not a census. /utlidar/cloud is among the UNSAMPLED ones. If a
subscription here ever comes up empty while the endpoint graph looks healthy,
this assumption is the first thing to check -- see CLAUDE.md gotcha 12, a healthy
endpoint graph does not imply traffic.

WHY depth=1 AND NOT A NAMED POLICY ENUM
--------------------------------------
QoSProfile(depth=1) is RELIABLE + KEEP_LAST(1) + VOLATILE by default, which is
exactly the measured publisher profile. Spelling the policies out would mean
naming enum members, and the short aliases (QoSReliabilityPolicy.RELIABLE) do not
exist in Foxy's rclpy -- only the RMW_QOS_POLICY_* spellings do. Relying on the
defaults is both correct here and portable across the two distros this package
has to build on.

WHY DURABILITY IS NEVER SET
---------------------------
Durability is VOLATILE on every measured endpoint: no late-joiner history, no
latched last value. A TRANSIENT_LOCAL request is incompatible with a VOLATILE
offer and receives NOTHING -- not stale data, nothing at all. That is an easy
reflex on topics that look like configuration or state, and it fails silently, so
this module never sets durability and the static safety test rejects any call
that does.

WHY depth=1 AND NOT depth=10
----------------------------
The robot's history is KEEP_LAST(1): a reliable writer with a one-deep queue
OVERWRITES rather than queues when the reader falls behind. Reliability buys
retransmission of the NEWEST sample, not a complete sequence. A deeper reader
queue cannot recover samples the writer already discarded, so it would only add
latency under load while suggesting a completeness we do not have. The adapter is
current-state polling, not an event log.
"""
from rclpy.qos import QoSProfile

#: Subscriptions to Go2 topics. Matches the measured publisher profile exactly.
GO2_INPUT_QOS = QoSProfile(depth=1)

#: Republished sensor streams under /go2/. Same shape, for the same reason: a
#: consumer that falls behind should get the newest cloud, not a backlog of stale
#: ones. Depth 1 makes "latest value" the contract instead of an accident.
GO2_OUTPUT_QOS = QoSProfile(depth=1)

#: Status and liveness. Depth 1 again, and deliberately NOT TRANSIENT_LOCAL even
#: though a latched last value is tempting for a health flag: a late subscriber
#: reading a latched "link ok" that was published before the link died would be
#: worse than reading nothing. A stale liveness flag is an anti-signal.
GO2_STATUS_QOS = QoSProfile(depth=1)
