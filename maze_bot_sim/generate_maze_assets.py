import os
import cv2
import numpy as np
import yaml

# Configuration
# 1 unit = 10cm = 0.1m
SCALE = 0.1 
MAP_RES = 0.01 # 1cm per pixel for map
IMG_SIZE = int(2.0 / MAP_RES) # 2.0m x 2.0m canvas
ORIGIN_X = -0.1 # Map origin offset
ORIGIN_Y = -0.1

# Colors
WHITE = 255
BLACK = 0
GRAY = 128

def create_blank_map():
    return np.full((IMG_SIZE, IMG_SIZE), 205, dtype=np.uint8) # 205 is unknown/light gray

def world_to_pixel(x, y):
    # x, y in meters
    # origin is at (-0.1, -0.1)
    px = int((x - ORIGIN_X) / MAP_RES)
    py = int((y - ORIGIN_Y) / MAP_RES)
    # Image coordinate y is inverted (top-left is 0,0)
    # But for PGM map server, usually (0,0) is bottom-left if we set origin correctly.
    # Let's draw in standard image coords where (0,0) is top-left, 
    # but ROS map server treats the image as starting from bottom-left.
    # To simplify, we will flip the image vertically at the end.
    return px, py

def draw_line(img, p1, p2, color, thickness_cm=2):
    px1, py1 = world_to_pixel(p1[0], p1[1])
    px2, py2 = world_to_pixel(p2[0], p2[1])
    thickness = int(thickness_cm * 0.01 / MAP_RES)
    cv2.line(img, (px1, py1), (px2, py2), color, thickness)

def generate_assets():
    # 1. Define Geometry (in meters)
    # Arena 18x18 units = 1.8m x 1.8m
    W = 1.8
    H = 1.8
    
    # Hard Obstacles (Walls)
    walls = [
        ((0,0), (W,0)), # Bottom
        ((W,0), (W,H)), # Right
        ((W,H), (0,H)), # Top
        ((0,H), (0,0)), # Left
        ((0.45, 0), (0.45, 0.45)), # GF
        ((0, 0.45), (0.45, 0.45))  # FE
    ]
    
    # Soft Obstacles (Tape)
    soft_obstacles = [
        ((0.45, 0.8), (0.45, 1.8)), # HI
        ((0.9, 0), (0.9, 1.35)),    # NK
        ((1.35, 0.45), (1.35, 1.8)) # LM
    ]

    # 2. Generate Maps
    # Map 1: Hard only
    img_stage1 = create_blank_map()
    # Fill free space (approximate the arena area)
    p_min = world_to_pixel(0, 0)
    p_max = world_to_pixel(W, H)
    cv2.rectangle(img_stage1, p_min, p_max, WHITE, -1)
    
    for p1, p2 in walls:
        draw_line(img_stage1, p1, p2, BLACK, thickness_cm=5) # Thicker for map safety
        
    # Map 2: Hard + Soft
    img_stage2 = img_stage1.copy()
    for p1, p2 in soft_obstacles:
        draw_line(img_stage2, p1, p2, BLACK, thickness_cm=5)

    # Flip for ROS map server convention
    img_stage1 = cv2.flip(img_stage1, 0)
    img_stage2 = cv2.flip(img_stage2, 0)

    # Save Maps
    # Use relative path or find package path dynamically if possible, but for generation script absolute is fine or relative to execution
    # We will assume this script is run from the workspace root or we hardcode the path for now as requested by user context
    base_path = '/home/yahboom/yahboomcar_ws/src/maze_bot_sim/maps'
    if not os.path.exists(base_path):
        os.makedirs(base_path)
        
    cv2.imwrite(os.path.join(base_path, 'maze_stage1.pgm'), img_stage1)
    cv2.imwrite(os.path.join(base_path, 'maze_stage2.pgm'), img_stage2)
    
    # Save YAMLs
    yaml_content = {
        'image': 'maze_stage1.pgm',
        'resolution': MAP_RES,
        'origin': [ORIGIN_X, ORIGIN_Y, 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196
    }
    
    with open(os.path.join(base_path, 'maze_stage1.yaml'), 'w') as f:
        yaml.dump(yaml_content, f)
        
    yaml_content['image'] = 'maze_stage2.pgm'
    with open(os.path.join(base_path, 'maze_stage2.yaml'), 'w') as f:
        yaml.dump(yaml_content, f)

    # 3. Generate Gazebo World
    sdf_walls = ""
    
    def make_box(name, x, y, w, h, is_visual=False):
        color = "Gazebo/Blue" if is_visual else "Gazebo/Grey"
        collision = ""
        if not is_visual:
            collision = f"""
        <collision name='collision'>
          <geometry>
            <box>
              <size>{w} {h} 0.5</size>
            </box>
          </geometry>
        </collision>"""
        
        return f"""
    <model name='{name}'>
      <pose frame=''>{x} {y} 0.25 0 -0 0</pose>
      <link name='link'>
        {collision}
        <visual name='visual'>
          <geometry>
            <box>
              <size>{w} {h} 0.5</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>{color}</name>
            </script>
          </material>
        </visual>
      </link>
      <static>1</static>
    </model>"""

    # Outer Walls
    thickness = 0.05
    sdf_walls += make_box("wall_bottom", W/2, -thickness/2, W + thickness*2, thickness)
    sdf_walls += make_box("wall_top", W/2, H + thickness/2, W + thickness*2, thickness)
    sdf_walls += make_box("wall_left", -thickness/2, H/2, thickness, H)
    sdf_walls += make_box("wall_right", W + thickness/2, H/2, thickness, H)
    
    # Inner Hard Walls
    # GF: (0.45, 0) to (0.45, 0.45) -> Center (0.45, 0.225), Size (thickness, 0.45)
    sdf_walls += make_box("wall_GF", 0.45, 0.225, thickness, 0.45)
    # FE: (0, 0.45) to (0.45, 0.45) -> Center (0.225, 0.45), Size (0.45, thickness)
    sdf_walls += make_box("wall_FE", 0.225, 0.45, 0.45, thickness)
    
    # Soft Obstacles (Visual Only)
    # HI: (0.45, 0.8) to (0.45, 1.8) -> Center (0.45, 1.3), Size (thickness, 1.0)
    sdf_walls += make_box("soft_HI", 0.45, 1.3, thickness, 1.0, is_visual=True)
    # NK: (0.9, 0) to (0.9, 1.35) -> Center (0.9, 0.675), Size (thickness, 1.35)
    sdf_walls += make_box("soft_NK", 0.9, 0.675, thickness, 1.35, is_visual=True)
    # LM: (1.35, 0.45) to (1.35, 1.8) -> Center (1.35, 1.125), Size (thickness, 1.35)
    sdf_walls += make_box("soft_LM", 1.35, 1.125, thickness, 1.35, is_visual=True)

    world_content = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="maze">
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://ground_plane</uri>
    </include>
    {sdf_walls}
  </world>
</sdf>
"""
    
    world_path = '/home/yahboom/yahboomcar_ws/src/maze_bot_sim/worlds/maze.world'
    if not os.path.exists(os.path.dirname(world_path)):
        os.makedirs(os.path.dirname(world_path))
        
    with open(world_path, 'w') as f:
        f.write(world_content)

if __name__ == "__main__":
    generate_assets()
    print("Maze assets generated successfully.")
