import rerun as rr
import numpy as np
from scipy.spatial.transform import Rotation as R
import struct
import sys
import os
import argparse

def get_terrain_color(t):
    stops = np.array([
        [0.1, 0.1, 0.3],  
        [0.0, 0.5, 0.8],  
        [0.1, 0.6, 0.1],  
        [0.8, 0.7, 0.2],  
        [0.5, 0.2, 0.1]   
    ])
    n_stops = len(stops)
    t = np.clip(t, 0.0, 1.0)
    idx = t * (n_stops - 1)
    low_idx = np.floor(idx).astype(int)
    high_idx = np.ceil(idx).astype(int)
    frac = idx - low_idx
    colors = (1 - frac[..., None]) * stops[low_idx] + frac[..., None] * stops[high_idx]
    return colors

def calculate_normals(grid_data, cell_size):
    dzdx, dzdy = np.gradient(grid_data, cell_size)
    normals = np.stack([-dzdx, -dzdy, np.ones_like(grid_data)], axis=-1)
    norm = np.linalg.norm(normals, axis=-1, keepdims=True)
    return (normals / norm).reshape(-1, 3)

def main():
    parser = argparse.ArgumentParser(description="Visualize DozeRL Simulation")
    parser.add_argument("--mode", type=str, choices=["eval", "kinematics"], default="eval", help="Visualization mode")
    args = parser.parse_args()

    rr.init(f"SoilSim_{args.mode.capitalize()}", spawn=True)
    
    header_format = "i22fif"
    header_size = struct.calcsize(header_format)
    
    bin_filename = "sim_test.bin" if args.mode == "kinematics" else "sim_out.bin"
    try:
        bin_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "out", bin_filename)
        f = open(bin_path, "rb")
    except FileNotFoundError:
        print(f"Error: {bin_path} not found.")
        return

    print(f"Loading binary simulation data into Rerun ({args.mode} mode)...")
    
    geom_format = "16f"
    geom_size = struct.calcsize(geom_format)
    geom_data = f.read(geom_size)
    if not geom_data:
        print("Error: Could not read geometry header.")
        return
        
    geom = struct.unpack(geom_format, geom_data)
    (loader_length, loader_width, loader_height,
     blade_height, blade_thickness, blade_width,
     track_length, track_width, track_height, track_gauge,
     arm_r, pivot_x, pivot_z, arm_y_left, arm_y_right,
     blade_pitch_length) = geom
    
    while True:
        header_data = f.read(header_size)
        if not header_data:
            break
            
        header = struct.unpack(header_format, header_data)
        step = header[0]
        (loader_x, loader_z, loader_y, yaw, pitch, roll,
         arm_height, vel_arm,
         blade_pitch_rel, vel_pitch,
         blade_roll_rel, vel_roll,
         blade_yaw_rel, vel_yaw,
         track_v_lin, track_v_rot) = header[1:17]
         
        (eff_linear, eff_rotational, eff_lift, 
         eff_pitch, eff_roll, eff_yaw) = header[17:23]
         
        grid_size = header[23]
        cell_size = header[24]
        
        grid_bytes = f.read(grid_size * grid_size * 4)
        if not grid_bytes: break
        grid_data = np.frombuffer(grid_bytes, dtype=np.float32).reshape(grid_size, grid_size)

        goal_bytes = f.read(grid_size * grid_size * 4)
        if not goal_bytes: break
        goal_data = np.frombuffer(goal_bytes, dtype=np.float32).reshape(grid_size, grid_size)
        
        rr.set_time("step_index", sequence=step)
        
        if args.mode == "eval":
            # Log telemetry
            rr.log("kinematics/effort/linear", rr.Scalars(eff_linear))
            rr.log("kinematics/effort/rotational", rr.Scalars(eff_rotational))
            rr.log("kinematics/effort/lift", rr.Scalars(eff_lift))
            rr.log("kinematics/effort/pitch", rr.Scalars(eff_pitch))
            rr.log("kinematics/effort/roll", rr.Scalars(eff_roll))
            rr.log("kinematics/effort/yaw", rr.Scalars(eff_yaw))

            rr.log("kinematics/velocity/linear", rr.Scalars(track_v_lin))
            rr.log("kinematics/velocity/rotational", rr.Scalars(track_v_rot))
            rr.log("kinematics/velocity/lift", rr.Scalars(vel_arm))
            rr.log("kinematics/velocity/pitch", rr.Scalars(vel_pitch))
            rr.log("kinematics/velocity/roll", rr.Scalars(vel_roll))
            rr.log("kinematics/velocity/yaw", rr.Scalars(vel_yaw))

        rr.log("kinematics/position/lift", rr.Scalars(arm_height))
        rr.log("kinematics/position/pitch", rr.Scalars(blade_pitch_rel))
        rr.log("kinematics/position/roll", rr.Scalars(blade_roll_rel))
        rr.log("kinematics/position/yaw", rr.Scalars(blade_yaw_rel))
        
        # Terrain Mesh
        normals = calculate_normals(grid_data, cell_size)
        light_dir = np.array([-1.0, -1.0, 1.5]); light_dir /= np.linalg.norm(light_dir)
        shade = np.clip(np.sum(normals * light_dir, axis=-1), 0.0, 1.0)
        shade = 0.4 + 0.6 * shade

        FIXED_MIN_Z, FIXED_MAX_Z = 0.5, 2.0
        t = np.clip((grid_data - FIXED_MIN_Z) / (FIXED_MAX_Z - FIXED_MIN_Z), 0.0, 1.0)
        vertex_colors = (get_terrain_color(t).reshape(-1, 3) * shade[..., None] * 255).astype(np.uint8)
        
        x, y = np.meshgrid(np.arange(grid_size) * cell_size, np.arange(grid_size) * cell_size, indexing='ij')
        vertices = np.stack([x, y, grid_data], axis=-1).reshape(-1, 3)
        
        if step == 0 or 'triangles' not in locals():
            indices = []
            for i in range(grid_size - 1):
                for j in range(grid_size - 1):
                    v0, v1, v2, v3 = i*grid_size+j, (i+1)*grid_size+j, (i+1)*grid_size+(j+1), i*grid_size+(j+1)
                    indices.extend([[v0, v1, v2], [v0, v2, v3]])
            triangles = np.array(indices, dtype=np.uint32)

        rr.log("world/terrain", rr.Mesh3D(
            vertex_positions=vertices, 
            vertex_colors=vertex_colors, 
            vertex_normals=normals,
            triangle_indices=triangles
        ))
        
        # Goal Mesh
        goal_normals = calculate_normals(goal_data, cell_size)
        shade_g = np.clip(np.sum(goal_normals * light_dir, axis=-1), 0.0, 1.0)
        shade_g = 0.5 + 0.5 * shade_g
        
        goal_vertices = np.stack([x, y, goal_data + 0.02], axis=-1).reshape(-1, 3)
        goal_base_color = np.array([0, 180, 255, 130]) 
        goal_colors = (goal_base_color[None, :3] * shade_g[..., None]).astype(np.uint8)
        goal_colors_with_alpha = np.column_stack([goal_colors, np.full(len(goal_colors), goal_base_color[3], dtype=np.uint8)])

        rr.log("world/goal", rr.Mesh3D(
            vertex_positions=goal_vertices,
            vertex_colors=goal_colors_with_alpha,
            vertex_normals=goal_normals,
            triangle_indices=triangles
        ))

        # Machine Visualization
        rot_chassis = R.from_euler('ZYX', [yaw, -pitch, roll], degrees=False).as_quat()
        
        rr.log("world/machine", rr.Transform3D(translation=[loader_x, loader_y, loader_z], rotation=rr.Quaternion(xyzw=rot_chassis)))
        rr.log("world/machine/chassis", rr.Boxes3D(
            half_sizes=[[loader_length/2.0, loader_width/2.0, loader_height/2.0]],
            centers=[[0.0, 0.0, loader_height/2.0]],
            colors=[[200, 200, 200]]
        ))
        
        rr.log("world/machine/tracks", rr.Boxes3D(
            half_sizes=[[track_length/2.0, track_width/2.0, track_height/2.0], [track_length/2.0, track_width/2.0, track_height/2.0]],
            centers=[[0.0, track_gauge/2.0, track_height/2.0], [0.0, -track_gauge/2.0, track_height/2.0]],
            colors=[[60, 60, 60], [60, 60, 60]]
        ))

        # Lift Arm Frame
        rot_arm = R.from_euler('Y', -arm_height, degrees=False).as_quat()
        rr.log("world/machine/lift_arm", rr.Transform3D(
            translation=[pivot_x, 0, pivot_z],
            rotation=rr.Quaternion(xyzw=rot_arm)
        ))
        rr.log("world/machine/lift_arm/mesh", rr.LineStrips3D(
            [
                [[0, arm_y_left, 0], [arm_r, arm_y_left, 0]],
                [[0, arm_y_right, 0], [arm_r, arm_y_right, 0]]
            ],
            radii=[0.08, 0.08],
            colors=[[150, 150, 150], [150, 150, 150]]
        ))

        # Pitch Link Frame
        rot_pitch = R.from_euler('Y', -blade_pitch_rel, degrees=False).as_quat()
        rr.log("world/machine/lift_arm/pitch_link", rr.Transform3D(
            translation=[arm_r, 0, 0],
            rotation=rr.Quaternion(xyzw=rot_pitch)
        ))
        
        # Pitch Links (attached from pitch joint to blade center)
        rr.log("world/machine/lift_arm/pitch_link/mesh", rr.LineStrips3D(
            [
                [[0, arm_y_left, 0], [blade_pitch_length, 0, blade_height/2.0]],
                [[0, arm_y_right, 0], [blade_pitch_length, 0, blade_height/2.0]]
            ],
            radii=[0.06, 0.06],
            colors=[[180, 180, 180], [180, 180, 180]]
        ))

        # Roll Frame
        rot_roll = R.from_euler('X', blade_roll_rel, degrees=False).as_quat()
        rr.log("world/machine/lift_arm/pitch_link/roll_link", rr.Transform3D(
            translation=[blade_pitch_length, 0, 0],
            rotation=rr.Quaternion(xyzw=rot_roll)
        ))

        # Yaw Frame
        rot_yaw = R.from_euler('Z', blade_yaw_rel, degrees=False).as_quat()
        rr.log("world/machine/lift_arm/pitch_link/roll_link/yaw_link", rr.Transform3D(
            translation=[0, 0, 0],
            rotation=rr.Quaternion(xyzw=rot_yaw)
        ))

        # Blade Mesh
        rr.log("world/machine/lift_arm/pitch_link/roll_link/yaw_link/blade", rr.Boxes3D(
            half_sizes=[[blade_thickness/2.0, blade_width/2.0, blade_height/2.0]],
            centers=[[0.0, 0.0, blade_height/2.0]],
            colors=[[255, 50, 50]]
        ))

    f.close()
    print("Done.")

if __name__ == "__main__":
    main()
