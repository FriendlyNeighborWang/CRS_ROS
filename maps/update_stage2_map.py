import cv2
import numpy as np
import yaml
import os

def update_map():
    base_path = '/home/yahboom/yahboomcar_ws/src/maze_bot_sim/maps'
    pgm_path = os.path.join(base_path, 'maze_stage1.pgm') # Use stage1 as base
    yaml_path = os.path.join(base_path, 'maze_stage2.yaml')
    out_pgm_path = os.path.join(base_path, 'maze_stage2.pgm')

    # Read YAML to get resolution and origin
    with open(yaml_path, 'r') as f:
        map_config = yaml.safe_load(f)
    
    resolution = map_config['resolution']
    origin = map_config['origin'] # [x, y, z]
    origin_x = origin[0]
    origin_y = origin[1]

    # Read PGM image
    # Note: cv2.imread might not handle P5 (binary) PGM correctly in all versions, 
    # but usually it does. If not, we use raw read.
    img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        print("Failed to read image with cv2, trying raw read")
        with open(pgm_path, 'rb') as f:
            header = b""
            while True:
                line = f.readline()
                header += line
                if len(line.strip().split()) == 1 and line.strip().isdigit() and int(line) == 255:
                    break # End of header
            data = f.read()
            img = np.frombuffer(data, dtype=np.uint8).reshape((200, 200))

    height, width = img.shape
    print(f"Map Size: {width}x{height}")

    # Helper to convert world to pixel
    def world_to_pixel(wx, wy):
        px = int((wx - origin_x) / resolution)
        # ROS map origin is bottom-left, Image origin is top-left
        # So we flip Y
        py = height - 1 - int((wy - origin_y) / resolution)
        return px, py

    # Walls to draw (World Coords)
    # 1. soft_HI: x=0.45, y=[0.8, 1.8], width=0.05
    # 2. soft_NK: x=0.90, y=[0.0, 1.35], width=0.05
    # 3. soft_LM: x=1.35, y=[0.45, 1.8], width=0.05
    
    walls = [
        {'x': 0.45, 'y_min': 0.8, 'y_max': 1.8},
        {'x': 0.90, 'y_min': 0.0, 'y_max': 1.35},
        {'x': 1.35, 'y_min': 0.45, 'y_max': 1.8}
    ]
    
    wall_thickness_m = 0.05
    wall_thickness_px = int(wall_thickness_m / resolution)

    # Draw walls (Black = 0 = Occupied)
    # In PGM: 0 is black (occupied), 255 is white (free), 205 is unknown usually.
    # We want to draw BLACK (0).
    
    for wall in walls:
        # Calculate start and end points
        # Since they are vertical walls
        pt1 = world_to_pixel(wall['x'], wall['y_min'])
        pt2 = world_to_pixel(wall['x'], wall['y_max'])
        
        # Draw line
        # Note: cv2 coordinates are (x, y)
        cv2.line(img, pt1, pt2, color=0, thickness=wall_thickness_px)
        print(f"Drew wall at x={wall['x']}")

    # Save updated map
    cv2.imwrite(out_pgm_path, img)
    print(f"Saved updated map to {out_pgm_path}")

if __name__ == '__main__':
    update_map()
