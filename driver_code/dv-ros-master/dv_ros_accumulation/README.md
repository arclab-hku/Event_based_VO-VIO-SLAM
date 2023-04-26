# DV ROS Accumulation

DV ROS Accumulation project provides two nodes: accumulation and edge_map nodes. These nodes can be used to accumulate
a stream of events into a stream of images representing the events. Stream of events is sliced into chunks by amount or
time depending on the configuration and is accumulated into a frame representation.

Nodes support dynamic reconfiguration, users are encouraged to test out different accumulation techniques.

# Accumulation node

Accumulation node performs event accumulation to reconstruct an image. It supports different decay functions and is
highly configurable.

# Edge map node

Edge map node generate and edge map from events instead of reconstructing an image. This representation can be
favorable for some computer vision applications. The edge map uses a simplified and optimized accumulation approach
for the edge extraction that is more efficient than regular accumulation.
