import numpy as np
import cv2
import yaml
import os

def create_maps():
    base_path = '/home/yahboom/yahboomcar_ws/src/maze_bot_sim/maps'
    if not os.path.exists(base_path):
        os.makedirs(base_path)

    # Map Parameters
    width_m = 2.0
    height_m = 2.0
    resolution = 0.01 # 1cm/pixel
    origin_x = -0.1
    origin_y = -0.1
    
    img_size_x = int(width_m / resolution)
    img_size_y = int(height_m / resolution)
    
    # Create blank white image (254)
    # 0 is black (obstacle), 254 is white (free), 205 is unknown
    img_stage1 = np.full((img_size_y, img_size_x), 254, dtype=np.uint8)
    img_stage2 = np.full((img_size_y, img_size_x), 254, dtype=np.uint8)

    def world_to_pixel(wx, wy):
        # wx, wy in meters
        # origin is at (-0.1, -0.1)
        # pixel x = (wx - origin_x) / res
        # pixel y = height - (wy - origin_y) / res  (Flip Y for image coords)
        px = int((wx - origin_x) / resolution)
        py = img_size_y - 1 - int((wy - origin_y) / resolution)
        return px, py

    def draw_line(img, p1, p2, thickness=2):
        # p1, p2 are (x, y) tuples in meters
        px1, py1 = world_to_pixel(p1[0], p1[1])
        px2, py2 = world_to_pixel(p2[0], p2[1])
        cv2.line(img, (px1, py1), (px2, py2), 0, thickness)

    def draw_rect(img, p1, p2, thickness=2):
        # Draw a rectangle defined by bottom-left p1 and top-right p2
        # We draw 4 lines
        draw_line(img, (p1[0], p1[1]), (p2[0], p1[1]), thickness) # Bottom
        draw_line(img, (p2[0], p1[1]), (p2[0], p2[1]), thickness) # Right
        draw_line(img, (p2[0], p2[1]), (p1[0], p2[1]), thickness) # Top
        draw_line(img, (p1[0], p2[1]), (p1[0], p1[1]), thickness) # Left

    # --- Hard Obstacles (Stage 1 & 2) ---
    # Outer Boundary (0,0) to (1.8, 1.8)
    # We draw it on both maps
    for img in [img_stage1, img_stage2]:
        # Outer Box
        draw_rect(img, (0.0, 0.0), (1.8, 1.8), thickness=3)
        
        # Internal Hard Walls
        # GF: (4.5, 0) -> (4.5, 4.5) => x=0.45, y=0..0.45
        draw_line(img, (0.45, 0.0), (0.45, 0.45), thickness=3)
        # FE: (4.5, 4.5) -> (0, 4.5) => y=0.45, x=0..0.45
        draw_line(img, (0.0, 0.45), (0.45, 0.45), thickness=3)

    # --- Soft Obstacles (Stage 2 Only) ---
    # LM: y=0.3, x=0.8..1.8
    draw_line(img_stage2, (0.8, 0.3), (1.8, 0.3), thickness=3)
    
    # M1F: x=0.45, y=0.45..0.75
    draw_line(img_stage2, (0.45, 0.45), (0.45, 0.75), thickness=3)
    
    # M1L1: y=0.75, x=0.45..1.45
    draw_line(img_stage2, (0.45, 0.75), (1.45, 0.75), thickness=3)
    
    # KN: y=1.05, x=0..0.6
    draw_line(img_stage2, (0.0, 1.05), (0.6, 1.05), thickness=3)
    
    # N1K1: y=1.15, x=1.2..1.8
    draw_line(img_stage2, (1.2, 1.15), (1.8, 1.15), thickness=3)
    
    # IH: y=1.4, x=0.6..1.5
    draw_line(img_stage2, (0.6, 1.4), (1.5, 1.4), thickness=3)
    
    # TS: x=0.15, y=1.25..1.7
    draw_line(img_stage2, (0.15, 1.25), (0.15, 1.7), thickness=3)
    
    # RS: y=1.7, x=0.15..0.5
    draw_line(img_stage2, (0.15, 1.7), (0.5, 1.7), thickness=3)

    # Save PGMs
    cv2.imwrite(os.path.join(base_path, 'maze_stage1.pgm'), img_stage1)
    cv2.imwrite(os.path.join(base_path, 'maze_stage2.pgm'), img_stage2)

    # Save YAMLs
    yaml_content = {
        'image': 'maze_stage1.pgm',
        'resolution': resolution,
        'origin': [origin_x, origin_y, 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196
    }
    
    with open(os.path.join(base_path, 'maze_stage1.yaml'), 'w') as f:
        yaml.dump(yaml_content, f)
        
    yaml_content['image'] = 'maze_stage2.pgm'
    with open(os.path.join(base_path, 'maze_stage2.yaml'), 'w') as f:
        yaml.dump(yaml_content, f)

    print("Maps generated successfully.")

if __name__ == '__main__':
    create_maps()
