#!/usr/bin/env python3
"""Depth-stream relay for perception-failure injection.

Forwards the live depth image to an output topic the nav node listens on.
Two injection knobs, both wall-clock driven so they work while sim runs:

  output topic   argv[1]  (default /camera_front/depth/relay)
  delay seconds  argv[2]  (default 0) — buffers frames and republishes them
                 LATER, keeping the ORIGINAL sim stamps, so the node's
                 observation age (sim_now - stamp) equals the injected
                 end-to-end latency.

Frames are pre-throttled to 5 Hz downstream (the nav node fuses at
perception_update_hz=5 anyway) to bound the delay buffer's memory.

Dies on SIGTERM/SIGINT: the orchestrator kills it mid-flight to simulate a
camera dropout; the stream simply stops while sim time keeps advancing.
"""
import heapq
import signal
import sys
import threading
import time

import rospy
from sensor_msgs.msg import Image

argv = rospy.myargv()
OUT_TOPIC = argv[1] if len(argv) > 1 else "/camera_front/depth/relay"
DELAY_S = float(argv[2]) if len(argv) > 2 else 0.0
IN_TOPIC = argv[3] if len(argv) > 3 else "/camera_front/depth/image_raw"
THROTTLE_S = 0.2  # 5 Hz downstream cap

pub = None
lock = threading.Lock()
heap = []  # (due_wall_s, seq, msg)
seq = [0]
last_forwarded = [None]


def on_image(msg):
    stamp = msg.header.stamp.to_sec()
    if last_forwarded[0] is not None and stamp - last_forwarded[0] < THROTTLE_S:
        return
    last_forwarded[0] = stamp
    if DELAY_S <= 0:
        pub.publish(msg)
        return
    with lock:
        seq[0] += 1
        heapq.heappush(heap, (time.monotonic() + DELAY_S, seq[0], msg))


def pump():
    while not rospy.is_shutdown():
        work = []
        with lock:
            while heap and heap[0][0] <= time.monotonic():
                work.append(heapq.heappop(heap))
        for _, _, msg in work:
            pub.publish(msg)
        time.sleep(0.01 if not work else 0.0)


def main():
    global pub
    rospy.init_node("depth_relay", disable_signals=True)
    signal.signal(signal.SIGTERM, lambda *_: rospy.signal_shutdown("killed"))
    pub = rospy.Publisher(OUT_TOPIC, Image, queue_size=2)
    rospy.Subscriber(IN_TOPIC, Image, on_image, queue_size=1,
                     buff_size=64 * 64 * 4 * 4)
    threading.Thread(target=pump, daemon=True).start()
    rospy.loginfo("depth_relay: %s -> %s delay=%.2fs", IN_TOPIC, OUT_TOPIC, DELAY_S)
    rospy.spin()
    print("DEPTH_RELAY_EXITED")


if __name__ == "__main__":
    main()
