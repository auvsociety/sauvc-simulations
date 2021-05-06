# Notes on understanding DSO

### The coordinate system
DSO follows standard pinhole camera coordinate system.<br>
The following table shows the mapping between the coordinate system axes:
| DSO(local) | Gazebo (wrt SAUVC round 2 position) | AUV\_v2(local) |
| --- | --- | --- |
| +X axis | +Y axis | +Y axis | 
| +Y axis | -Z axis | -Z axis |
| +Z axis | -X axis | +X axis |

[Back to Home](./Home.md)