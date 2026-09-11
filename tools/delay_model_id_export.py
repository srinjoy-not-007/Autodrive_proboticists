#!/usr/bin/env python3
"""Export a rosbag2 sqlite bag to flat CSVs (run inside the devkit container)."""
import csv, os, sqlite3, sys
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import Point
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32

bag, out = sys.argv[1], sys.argv[2]
os.makedirs(out, exist_ok=True)
db = [f for f in os.listdir(bag) if f.endswith('.db3')][0]
conn = sqlite3.connect(os.path.join(bag, db))
topics = {n: (i, t) for i, n, t in conn.execute('SELECT id, name, type FROM topics')}
for n, (i, t) in sorted(topics.items()):
    print(n, t)

NS = '/autodrive/roboracer_1'
def rows(topic):
    tid = topics[topic][0]
    return conn.execute('SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp', (tid,))

def dump(name, topic, cls, fields):
    if topic not in topics:
        print('missing', topic); return
    with open(os.path.join(out, name + '.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t'] + [k for k, _ in fields])
        for ts, data in rows(topic):
            m = deserialize_message(bytes(data), cls)
            w.writerow(['%.9f' % (ts * 1e-9)] + ['%.9g' % fn(m) for _, fn in fields])

dump('ips', NS + '/ips', Point, [('x', lambda m: m.x), ('y', lambda m: m.y), ('z', lambda m: m.z)])
dump('imu', NS + '/imu', Imu, [
    ('qx', lambda m: m.orientation.x), ('qy', lambda m: m.orientation.y),
    ('qz', lambda m: m.orientation.z), ('qw', lambda m: m.orientation.w),
    ('wz', lambda m: m.angular_velocity.z),
    ('ax', lambda m: m.linear_acceleration.x), ('ay', lambda m: m.linear_acceleration.y),
    ('stamp', lambda m: m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)])
for wheel in ('left_encoder', 'right_encoder'):
    dump(wheel, NS + '/' + wheel, JointState, [
        ('pos', lambda m: m.position[0] if m.position else float('nan')),
        ('vel', lambda m: m.velocity[0] if m.velocity else float('nan')),
        ('stamp', lambda m: m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)])
dump('steering', NS + '/steering', Float32, [('v', lambda m: m.data)])
dump('throttle', NS + '/throttle', Float32, [('v', lambda m: m.data)])
for extra, topic in (('steering_command', NS + '/steering_command'),
                     ('throttle_command', NS + '/throttle_command'),
                     ('cte', '/pit_crew/cte')):
    dump(extra, topic, Float32, [('v', lambda m: m.data)])
print('done ->', out)
