# Illustration generation

Generated with the built-in image_gen tool. This is conceptual artwork, not a simulation result.

## Initial prompt

Use case: infographic-diagram.
Asset type: polished landscape GitHub README hero/technical figure for ALPS-SLAM, approximately 16:9.
Primary request: Create a beautiful, scientifically credible illustrated overview of mobile robot autonomous exploration with LIO-SAM and CMU TARE. A custom editorial science illustration, generous white space, precise typography, restrained navy blue and teal accents on a white background, thin clean lines, subtle depth, sophisticated academic publication quality. Not a generic box-and-arrow flowchart.
Composition: top title "ALPS-SLAM", subtitle "自主探索与导航". Large central illustration of an isometric indoor warehouse/corridor environment with a small realistic wheeled lidar robot. Show partially observed areas as light gray floor with dark slate walls, unexplored space softly shaded, sparse blue lidar point cloud near observed walls, blue frontier dots along the transition, a teal exploration route strictly within corridors and a short orange local avoidance segment. Clearly illustrative, not an experimental screenshot.
Arrange five beautifully spaced technical modules around/below the central environment, each with a meaningful miniature illustration and readable short bilingual labels:
"01  LIO-SAM" / "定位与建图" with lidar and IMU sensing illustration.
"02  TARE" / "探索目标生成" with frontier and exploration waypoint illustration.
"03  Terrain Analysis" / "可通行性分析" with ground/obstacle separation illustration.
"04  Local Planner" / "局部规划与避障" with candidate curved paths near an obstacle.
"05  ROS Integration" / "路径跟踪与底盘控制" with wheel/control illustration.
Subtle deliberate connectors convey: LIO-SAM supplies pose and registered clouds to both TARE and terrain analysis; terrain analysis informs TARE and local planner; TARE sends waypoints to local planner; local planner connects to ROS control; robot motion yields new observations. Do not make terrain analysis appear downstream of TARE. Keep connectors sparse and legible, no spaghetti.
Small bottom legend with three symbols and exact labels: "已探索区域", "待探索区域", "规划路径". Small footer "技术路线示意 · Conceptual illustration".
Typography: excellent readable sans-serif Chinese and Latin, short labels only, exact algorithm names. Strong visual hierarchy, no tiny paragraphs.
Avoid: university logos, institutional affiliations, award claims, performance metrics, test-success claims, FAR, FAST-LIO, YOLO, humanoids, glossy neon sci-fi, heavy gradient panels, clutter, stock presentation clipart, watermarks.

## Final refinement prompt

Edit this ALPS-SLAM illustration. Preserve the beautiful central warehouse scene, robot, point clouds, title, five numbered module cards and their miniature illustrations, colors, fonts, bottom legend and conceptual-illustration footer.
Make exactly these changes:
1. Remove ALL arrows and connector lines BETWEEN the five module cards and ALL wiring connectors from cards up into the warehouse. The cards are five module illustrations, NOT a sequential dataflow diagram. Keep arrows INSIDE the local-planner miniature, robot wheel movement marks, and route arrows INSIDE the warehouse.
2. Remove the promotional text at upper left ("让机器人..." and English slogan) and upper right ("未知空间..." and English slogan), leaving tasteful white space.
3. Keep the central title "ALPS-SLAM" and subtitle "自主探索与导航".
4. Replace small card 05 label "路径跟踪与底盘控制" with exactly "路径跟踪与底盘控制" (preserve existing correct label).
Do not add any new connectors, slogans, metrics or claims. Keep the five card order and all Chinese/English algorithm labels unchanged. Crisp high-resolution landscape academic project illustration.

