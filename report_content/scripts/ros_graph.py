#!/usr/bin/env python3
"""Draw the live ROS graph (nodes and topics, like rqt_graph) with graphviz.

  ros_graph.py OUT_BASENAME [--skip node1,node2]   -> OUT.dot, OUT.png, OUT.txt

OUT.txt lists every node with its published and subscribed topics.
"""
import subprocess
import sys
import time

import rclpy

NOISE_TOPICS = {'/rosout', '/parameter_events', '/clock'}
NOISE_SUFFIX = ('/transition_event', '/_action/status', '/_action/feedback', 'bond')
NOISE_NODES = ('ros2cli', 'ros_graph', '_ros2cli', 'launch_ros', 'transform_listener_impl')


def keep_topic(t):
    return t not in NOISE_TOPICS and not t.endswith(NOISE_SUFFIX)


def main():
    out = sys.argv[1]
    skip = set(sys.argv[3].split(',')) if len(sys.argv) > 3 and sys.argv[2] == '--skip' else set()
    rclpy.init()
    node = rclpy.create_node('ros_graph')
    time.sleep(3.0)  # discovery
    nodes = []
    for name, ns in node.get_node_names_and_namespaces():
        if name.startswith(NOISE_NODES) or name in skip:
            continue
        pubs = [t for t, _ in node.get_publisher_names_and_types_by_node(name, ns) if keep_topic(t)]
        subs = [t for t, _ in node.get_subscriber_names_and_types_by_node(name, ns) if keep_topic(t)]
        nodes.append((('' if ns == '/' else ns) + '/' + name, sorted(pubs), sorted(subs)))
    nodes.sort()

    with open(out + '.txt', 'w') as f:
        for n, pubs, subs in nodes:
            f.write('%s\n  publishes:  %s\n  subscribes: %s\n' % (n, ', '.join(pubs) or '-',
                                                                 ', '.join(subs) or '-'))
    used = {}
    for n, pubs, subs in nodes:
        for t in pubs:
            used.setdefault(t, [set(), set()])[0].add(n)
        for t in subs:
            used.setdefault(t, [set(), set()])[1].add(n)
    lines = ['digraph G {', '  rankdir=LR; splines=true; nodesep=0.25; ranksep=0.9;',
             '  node [fontname="Helvetica", fontsize=11];']
    for n, _, _ in nodes:
        lines.append('  "%s" [shape=ellipse, style=filled, fillcolor="#cfe3ff"];' % n)
    for t, (p, s) in sorted(used.items()):
        if not p or not s:
            continue  # only topics that connect two nodes, as rqt_graph's default view
        lines.append('  "T%s" [label="%s", shape=box, style=filled, fillcolor="#fff2c6"];' % (t, t))
        for n in sorted(p):
            lines.append('  "%s" -> "T%s";' % (n, t))
        for n in sorted(s):
            lines.append('  "T%s" -> "%s";' % (t, n))
    lines.append('}')
    open(out + '.dot', 'w').write('\n'.join(lines) + '\n')
    subprocess.run(['dot', '-Tpng', '-Gdpi=130', out + '.dot', '-o', out + '.png'], check=True)
    print('wrote %s.png (%d nodes)' % (out, len(nodes)))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
