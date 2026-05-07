import rerun as rr
import numpy as np
from scipy.spatial.transform import Rotation as R
import struct

def get_terrain_color(t):
    """
    Perceptually rich colormap (Turbo-like) to make slight elevation 
    differences stand out across the full spectrum.
    """
    # Multi-stop linear interpolation for a vibrant palette:
    # Deep Blue -> Cyan -> Green -> Yellow -> Red
    stops = np.array([
        [0.0, 0.0, 0.5],  # Deep Blue
        [0.0, 0.5, 1.0],  # Cyan
        [0.0, 0.8, 0.0],  # Green
        [1.0, 1.0, 0.0],  # Yellow
        [1.0, 0.0, 0.0]   # Red
    ])
    
    n_stops = len(stops)
    t = np.clip(t, 0.0, 1.0)
    
    # Calculate stop indices
    idx = t * (n_stops - 1)
    low_idx = np.floor(idx).astype(int)
    high_idx = np.ceil(idx).astype(int)
    frac = idx - low_idx
    
    # Interpolate
    colors = (1 - frac[..., None]) * stops[low_idx] + frac[..., None] * stops[high_idx]
    return colors

def main():
    rr.init("SoilSim", spawn=True)
    
    # New Binary Header Format: 
    # int step (i)
    # 16 floats (16f): Pose(6), Joints(8), Tracks(2)
    # int grid_size (i)
    # float cell_size (f)
    header_format = "i16fif"
    header_size = struct.calcsize(header_format)
    
    try:
        f = open("out/sim_out.bin", "rb")
    except FileNotFoundError:
        print("Error: sim_out.bin not found. Did you run 'make sim && ./sim'?")
        return

    print("Loading binary simulation data into Rerun...")
    
    while True:
        header_data = f.read(header_size)
        if not header_data:
            break
            
        header = struct.unpack(header_format, header_data)
        step = header[0]
        (loader_x, loader_z, lat_pos, yaw, pitch, roll,
         arm_height, vel_arm,
         blade_pitch_rel, vel_pitch,
         blade_roll_rel, vel_roll,
         blade_yaw_rel, vel_yaw,
         track_v_lin, track_v_rot) = header[1:17]
        grid_size = header[17]
        cell_size = header[18]
        
        # Read the grid of floats
        grid_bytes = f.read(grid_size * grid_size * 4)
        if not grid_bytes:
            break
        grid_data = np.frombuffer(grid_bytes, dtype=np.float32).reshape(grid_size, grid_size)
        
        rr.set_time("step_index", sequence=step)
        
        # 1. Calculate Hillshading (Shaded Relief)
        # This is the most effective way to see small height variations
        # np.gradient returns (grad_axis0, grad_axis1) -> (dzdx, dzdy)
        dzdx, dzdy = np.gradient(grid_data, cell_size)
        
        # Light source from the top-left (machine start area)
        light_dir = np.array([-1.0, -1.0, 1.5]) 
        light_dir /= np.linalg.norm(light_dir)
        
        # Surface normals
        mag = np.sqrt(dzdx**2 + dzdy**2 + 1.0)
        nx, ny, nz = -dzdx/mag, -dzdy/mag, 1.0/mag
        
        # Lambertian shading + Ambient
        shade = (nx * light_dir[0] + ny * light_dir[1] + nz * light_dir[2])
        shade = np.clip(shade, 0.0, 1.0)
        shade = 0.3 + 0.7 * shade # 30% ambient light
        
        # 2. Enhanced Colormapping
        FIXED_MIN_Z = 0.5
        FIXED_MAX_Z = 2.0
        z_range = FIXED_MAX_Z - FIXED_MIN_Z
        t = np.clip((grid_data - FIXED_MIN_Z) / z_range, 0.0, 1.0)
        
        base_colors = get_terrain_color(t)
        
        # Apply hillshade to colors
        final_colors = (base_colors * shade[..., None] * 255).astype(np.uint8)
        vertex_colors = final_colors.reshape(-1, 3)
        
        # Create vertices
        x, y = np.meshgrid(np.arange(grid_size) * cell_size, np.arange(grid_size) * cell_size, indexing='ij')
        vertices = np.stack([x, y, grid_data], axis=-1).reshape(-1, 3)
        
        # Create triangle indices (only once)
        if step == 0 or 'triangles' not in locals():
            indices = []
            for i in range(grid_size - 1):
                for j in range(grid_size - 1):
                    v0 = i * grid_size + j
                    v1 = (i + 1) * grid_size + j
                    v2 = (i + 1) * grid_size + (j + 1)
                    v3 = i * grid_size + (j + 1)
                    indices.append([v0, v1, v2])
                    indices.append([v0, v2, v3])
            triangles = np.array(indices, dtype=np.uint32)

        rr.log("world/terrain", rr.Mesh3D(
            vertex_positions=vertices,
            vertex_colors=vertex_colors,
            triangle_indices=triangles
        ))
        
        # Machine dimensions
        loader_length = 2.5
        loader_width = 1.8
        loader_height = 2.5
        blade_height = 1.0
        blade_thickness = 0.2
        blade_width = 2.2 # Fixed width for visualization
        
        rot_chassis = R.from_euler('ZYX', [yaw, -pitch, roll], degrees=False).as_quat()
        
        # loader_lat is passed in the header as lat_pos (header[3])
        loader_lat = lat_pos 
        
        rr.log("world/machine", rr.Transform3D(
            translation=[loader_x, loader_lat, loader_z + loader_height / 2.0],
            rotation=rr.Quaternion(xyzw=rot_chassis)
        ))
        
        rr.log("world/machine/chassis", rr.Boxes3D(
            half_sizes=[[loader_length / 2.0, loader_width / 2.0, loader_height / 2.0]],
            colors=[[200, 200, 200]] # Greyscale machine to pop against colored terrain
        ))
        
        # Blade is attached to the machine, positioned in front
        blade_local_x = 2.0
        blade_local_z = arm_height - loader_height / 2.0 + blade_height / 2.0
        
        # Apply blade roll, pitch, and yaw relative to chassis
        rot_blade = R.from_euler('ZYX', [blade_yaw_rel, -blade_pitch_rel, blade_roll_rel], degrees=False).as_quat()

        rr.log("world/machine/blade_hinge", rr.Transform3D(
            translation=[blade_local_x, 0, blade_local_z],
            rotation=rr.Quaternion(xyzw=rot_blade)
        ))

        rr.log("world/machine/blade_hinge/mesh", rr.Boxes3D(
            half_sizes=[[blade_thickness / 2.0, blade_width / 2.0, blade_height / 2.0]], 
            colors=[[255, 50, 50]]
        ))

    f.close()
    print("Done.")

if __name__ == "__main__":
    main()
