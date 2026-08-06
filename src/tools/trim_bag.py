"""Trim a rosbag2 (sqlite3) to the first N seconds.

ROS 2 Humble's `ros2 bag play` has no duration flag, and our recordings can
run long, so slice them offline instead.

Usage: python3 trim_bag.py <in_bag_dir> <out_bag_dir> <seconds> [start_offset]
Then run `ros2 bag reindex <out_bag_dir>` (this script does it if available).
"""

import os
import shutil
import sqlite3
import subprocess
import sys


def main():
    src, dst, secs = sys.argv[1], sys.argv[2], float(sys.argv[3])
    offset = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0

    db_files = [f for f in os.listdir(src) if f.endswith('.db3')]
    if not db_files:
        sys.exit(f'no .db3 found in {src}')
    os.makedirs(dst, exist_ok=True)
    out_db = os.path.join(dst, os.path.basename(dst) + '_0.db3')
    shutil.copy(os.path.join(src, db_files[0]), out_db)

    conn = sqlite3.connect(out_db)
    t0 = conn.execute('SELECT MIN(timestamp) FROM messages').fetchone()[0]
    lo = t0 + int(offset * 1e9)
    hi = lo + int(secs * 1e9)
    before = conn.execute('SELECT COUNT(*) FROM messages').fetchone()[0]
    conn.execute('DELETE FROM messages WHERE timestamp < ? OR timestamp > ?',
                 (lo, hi))
    conn.commit()
    after = conn.execute('SELECT COUNT(*) FROM messages').fetchone()[0]
    conn.execute('VACUUM')
    conn.commit()
    conn.close()
    print(f'kept {after}/{before} messages ({secs}s from +{offset}s)')

    meta = os.path.join(dst, 'metadata.yaml')
    if os.path.exists(meta):
        os.remove(meta)
    try:
        subprocess.run(['ros2', 'bag', 'reindex', dst], check=False,
                       capture_output=True)
        print(f'reindexed {dst}')
    except FileNotFoundError:
        print(f'run: ros2 bag reindex {dst}')


if __name__ == '__main__':
    main()
