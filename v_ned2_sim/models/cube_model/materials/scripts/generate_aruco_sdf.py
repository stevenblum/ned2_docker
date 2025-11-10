#!/usr/bin/env python3
import sys # <--- ADD THIS LINE
import cv2 # If used in imports remains the same
from pathlib import Path # <--- ADD THIS LINE if not present

def generate_sdf(name, start_id, size, tex_root, x=0, y=0, z=0.05, yaw=0):
    h = size / 2.0; t = 0.01
    ids = [start_id + i for i in range(5)]
    script_path = Path(tex_root) / "models" / "cube_model" / "materials" / "scripts"
    tex_path = Path(tex_root) / "models" / "cube_model" / "materials" / "textures"

    def material_block(material_name):
        # Use double quotes inside the function
        return (f"<material><script>"
                f"<uri>file://{script_path}</uri>"
                f"<uri>file://{tex_path}</uri>"
                f"<name>{material_name}</name>"
                "</script></material>")

    # Use a clean f-string for the main template, handle inertia correctly
    # Use double quotes everywhere in the XML attributes!
    sdf = '<?xml version="1.0"?>'
    sdf += f'<sdf version="1.6"><model name="{name}">'
    sdf += f'<pose>{x} {y} {z} 0 0 {yaw}</pose><link name="{name}_link">'
    
    # Inertial properties
    sdf += '<inertial><mass>0.05</mass><inertia><ixx>0.000020833</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.000020833</iyy><iyz>0</iyz><izz>0.000020833</izz></inertia></inertial>'
    
    # Collision
    sdf += f'<collision name="collision"><geometry><box><size>{size} {size} {size}</size></box></geometry></collision>'

    # Main body visual
    sdf += f'<visual name="body_visual"><geometry><box><size>{size} {size} {size}</size></box></geometry><material><ambient>0.95 0.95 0.95 1</ambient><diffuse>0.95 0.95 0.95 1</diffuse></material></visual>'

    # Faces using material names (using list index to avoid syntax errors)
    sdf += f'<visual name="face_xp"><pose>{h + t/2} 0 0 0 0 0</pose><geometry><box><size>{t} {size} {size}</size></box></geometry>{material_block(f"aruco_{ids[0]}")}</visual>'
    sdf += f'<visual name="face_xm"><pose>{-h - t/2} 0 0 0 0 0</pose><geometry><box><size>{t} {size} {size}</size></box></geometry>{material_block(f"aruco_{ids[1]}")}</visual>'
    sdf += f'<visual name="face_yp"><pose>0 {h + t/2} 0 0 0 0</pose><geometry><box><size>{size} {t} {size}</size></box></geometry>{material_block(f"aruco_{ids[2]}")}</visual>'
    sdf += f'<visual name="face_ym"><pose>0 {-h - t/2} 0 0 0 0</pose><geometry><box><size>{size} {t} {size}</size></box></geometry>{material_block(f"aruco_{ids[3]}")}</visual>'
    sdf += f'<visual name="face_zp"><pose>0 0 {h + t/2} 0 0 0</pose><geometry><box><size>{size} {size} {t}</size></box></geometry>{material_block(f"aruco_{ids[4]}")}</visual>'
    
    # The bottom face uses the blank material
    sdf += f'<visual name="face_zm"><pose>0 0 {-h - t/2} 0 0 0</pose><geometry><box><size>{size} {size} {t}</size></box></geometry>{material_block("blank")}</visual>'
    sdf += "</link></model></sdf>"
    return sdf

if __name__ == "__main__":
    # ... (Keep your __main__ argument parsing logic as is) ...
    args = sys.argv[1:]
    if len(args) < 4: print("Usage: python3 generate_aruco_sdf.py NAME START_ID SIZE TEX_ROOT [X Y Z YAW]"); sys.exit(1)
    name = args[0]; start_id = int(args[1]); size = float(args[2]); tex_root = args[3]
    x = float(args[4]) if len(args) > 4 else 0.0; y = float(args[5]) if len(args) > 5 else 0.0
    z = float(args[6]) if len(args) > 6 else 0.05; yaw = float(args[7]) if len(args) > 7 else 0.0
    sdf = generate_sdf(name, start_id, size, tex_root, x, y, z, yaw)
    print(sdf)

