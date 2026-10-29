# Small V target: MiR reference profile

Source: Mobile Industrial Robots A/S, **MiR Fleet Enterprise Documentation**, January
2025, version 1.2, printed pages 53–54. The publicly accessible PDF is hosted by a
MiR distributor; it preserves manufacturer attribution. MiR's official support portal
requires login.

[Manufacturer-authored PDF, mirrored by JK](https://jk.de/media/a2/50/ce/1738939330/mir_fleet_enterprise_documentation_1.2_en.pdf?ts=1738939330#page=53)

| Parameter | Source specification | Applied model |
|---|---|---|
| Each arm | 150 ±1 mm | `arm_length: 0.15` m |
| Interior opening | 120 ±1 degrees | `opening_angle_deg: 120` |
| Physical height | At least 100 mm | 100 mm |
| Overall width | Derived, not printed plan dimension | 259.808 mm |
| Apex-to-mouth depth | Derived, not printed plan dimension | 75 mm |
| Finish | Matte neutral paint | Ordinary non-retroreflective simulator surface |

Width is `2 L sin(alpha/2)`, depth is `L cos(alpha/2)` for ideal zero-thickness
arms. The document does not specify a universal plate thickness. Do not interpret
these as an industry standard or as MiR's complete docking performance specification.

Page 53's 200 mm elevation dimension denotes the LiDAR **scan centerline**, not the
bottom of the plate. The physical target spans around that line. Our AMR700D front
LiDAR is at **263 mm**, so the simulation plate occupies **213–313 mm** and the
published opening-face midpoint is at **263 mm**. This height is configured, not estimated from 2D
ranges.

Shipped ROS YAML uses this small profile, a 45 mm minimum arm support, 10 mm endpoint
margin and 40 mm association gate. These extraction settings are estimator engineering
choices, not MiR-specified tolerances. Geometry tolerances above remain separate from
sensor range noise and association gates.

The previous 500 mm-arm, 70 degree, 2 m-tall simulator object was a synthetic stress
fixture. Its legacy tests remain available; they are not evidence for the small profile.
Small-profile evidence and failures/operating range are recorded separately in
`reports/mir_v150/`.

Verified source PDF SHA256:
`4b4693e227812f8fa81bbfb4f95591db15f8349a7c0ad1f1a01fb2d31f16331c`.


The public pose origin is the midpoint of the two arm tips (opening plane), not
the recessed apex. For this profile it is 75 mm toward the opening from the apex.
That geometric shift and its yaw-dependent covariance are computed automatically;
controller offsets are measured from the physical opening face. The internal
model and acquisition `target_x/y/yaw` prior still refer to the apex/opening axis.
