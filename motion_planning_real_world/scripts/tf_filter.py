#!/usr/bin/env python3
import rosbag
import math
import os
import sys
import rospy

def is_valid_quaternion(q, tolerance=0.05):
    # Check if quaternion has no NaNs/Infs and norm is close to 1
    norm = math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
    if any(math.isnan(v) or math.isinf(v) for v in [q.x, q.y, q.z, q.w]):
        return False
    if abs(norm - 1.0) > tolerance:
        return False
    return True

def filter_tf_messages(msg):
    clean_transforms = []
    for t in msg.transforms:
        q = t.transform.rotation
        if is_valid_quaternion(q):
            clean_transforms.append(t)
        else:
            print(f"⚠️ Dropping invalid TF: {t.header.frame_id} -> {t.child_frame_id} "
                  f"(q=({q.x:.3f}, {q.y:.3f}, {q.z:.3f}, {q.w:.3f}))")
    msg.transforms = clean_transforms
    return msg if clean_transforms else None

def main(input_bag, output_bag):
    if not os.path.exists(input_bag):
        print(f"Input bag file {input_bag} not found.")
        sys.exit(1)

    with rosbag.Bag(output_bag, 'w') as outbag:
        for topic, msg, t in rosbag.Bag(input_bag).read_messages():
            if topic in ['/tf', '/tf_static']:
                filtered_msg = filter_tf_messages(msg)
                if filtered_msg:
                    outbag.write(topic, filtered_msg, t)
            else:
                outbag.write(topic, msg, t)

    print(f"\n✅ Done. Cleaned bag saved as: {output_bag}")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: tf_filter.py input.bag output_filtered.bag")
        sys.exit(1)
    main(sys.argv[1], sys.argv[2])

