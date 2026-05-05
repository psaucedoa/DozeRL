import rerun as rr
import numpy as np
from scipy.spatial.transform import Rotation as R
import struct

def main():
    rr.init("SoilSim", spawn=True)
    
    # Binary Header Format: int, 10 floats, 1 int, 1 float
    header_format = "i10fif"
    header_size = struct.calcsize(header_format)
    
    try:
        f = open("sim_out.bin", "rb")
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
        bx, by, bwidth, pitch, roll, loader_x, loader_z, lat_pos, yaw, blade_roll_rel = header[1:11]
        grid_size = header[11]
        cell_size = header[12]
        
        # Read the grid of floats
        grid_bytes = f.read(grid_size * grid_size * 4)
        grid_data = np.frombuffer(grid_bytes, dtype=np.float32).reshape(grid_size, grid_size)
        
        rr.set_time("step_index", sequence=step)
        
        # Log Heightmap as a 3D Mesh
        min_z, max_z = grid_data.min(), grid_data.max()
        z_range = max_z - min_z if max_z > min_z else 1.0
        
        # Normalize heights for coloring
        t = (grid_data - min_z) / z_range
        r = (t * 255).astype(np.uint8)
        g = ((1.0 - np.abs(2.0*t - 1.0)) * 50).astype(np.uint8)
        b = ((1.0 - t) * 255).astype(np.uint8)
        vertex_colors = np.stack([r, g, b], axis=-1).reshape(-1, 3)
        
        # Create vertices
        x, y = np.meshgrid(np.arange(grid_size) * cell_size, np.arange(grid_size) * cell_size, indexing='ij')
        vertices = np.stack([x, y, grid_data], axis=-1).reshape(-1, 3)
        
        # Create triangle indices
        if step == 0 or step == -1:
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
        loader_length = 2.0
        loader_width = 1.5
        loader_height = 1.0
        blade_height = 0.5
        blade_thickness = 0.2
        
        # Chassis transform: translate by loader_x/lat_pos, rotate by yaw, pitch, roll
        # Note: Scipy from_euler applies rotations. The order 'xyz' implies roll, pitch, yaw
        # Rerun uses right-handed coordinates: +X Forward, +Y Left (or Right depending on setup), +Z Up.
        # Yaw is around Z. Pitch is around Y. Roll is around X.
        rot_chassis = R.from_euler('ZYX', [yaw, -pitch, roll], degrees=False).as_quat()
        
        rr.log("world/machine", rr.Transform3D(
            translation=[loader_x, lat_pos, loader_z + loader_height / 2.0],
            rotation=rr.Quaternion(xyzw=rot_chassis)
        ))
        
        rr.log("world/machine/chassis", rr.Boxes3D(
            half_sizes=[[loader_length / 2.0, loader_width / 2.0, loader_height / 2.0]],
            colors=[[50, 100, 255]]
        ))
        
        # Blade is attached to the machine, positioned in front
        blade_local_x = 1.5
        blade_local_z = -loader_height / 2.0 - 0.2 + blade_height / 2.0
        
        # Apply blade roll relative to chassis (rotation around X axis)
        rot_blade = R.from_euler('X', [blade_roll_rel], degrees=False).as_quat()

        rr.log("world/machine/blade_hinge", rr.Transform3D(
            translation=[blade_local_x, 0, blade_local_z],
            rotation=rr.Quaternion(xyzw=rot_blade)
        ))

        rr.log("world/machine/blade_hinge/mesh", rr.Boxes3D(
            half_sizes=[[blade_thickness / 2.0, bwidth / 2.0, blade_height / 2.0]], 
            colors=[[255, 50, 50]]
        ))

    f.close()
    print("Done.")

if __name__ == "__main__":
    main()
