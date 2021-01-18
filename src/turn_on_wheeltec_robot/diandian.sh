#!/bin/sh
rostopic pub -1 /clicked_point geometry_msgs/PointStamped  '{header: {frame_id: 'map'},point: {x: -15.5,y: 1.9,z: 0}}'
rostopic pub -1 /clicked_point geometry_msgs/PointStamped  '{header: {frame_id: 'map'},point: {x: -18.5,y: 20.5,z: 0}}'
rostopic pub -1 /clicked_point geometry_msgs/PointStamped  '{header: {frame_id: 'map'},point: {x: -22.2,y: 20.3,z: 0}}'
rostopic pub -1 /clicked_point geometry_msgs/PointStamped  '{header: {frame_id: 'map'},point: {x: -38.9,y: 19,z: 0}}'
rostopic pub -1 /clicked_point geometry_msgs/PointStamped  '{header: {frame_id: 'map'},point: {x: -38.0,y: 16.95,z: 0}}'
rostopic pub -1 /clicked_point geometry_msgs/PointStamped  '{header: {frame_id: 'map'},point: {x: -40.0,y: 5.5,z: 0}}'
rostopic pub -1 /clicked_point geometry_msgs/PointStamped  '{header: {frame_id: 'map'},point: {x: -17.5,y: 2.5,z: 0}}'
