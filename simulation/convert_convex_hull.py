import coacd
import trimesh
import numpy as np

# https://github.com/SarahWeiii/CoACD

def main():
    INPUT = "mesh/mug.stl"
    OUTPUT = "mesh/mug.obj"

    mesh = trimesh.load(INPUT, force="mesh")
    mesh = coacd.Mesh(mesh.vertices, mesh.faces)
    result = coacd.run_coacd(mesh)  # a list of convex hulls.ls
    mesh_parts = []
    for vs, fs in result:
        mesh_parts.append(trimesh.Trimesh(vs, fs))

    scene = trimesh.Scene()
    np.random.seed(0)
    for p in mesh_parts:
        p.visual.vertex_colors[:, :3] = (np.random.rand(3) * 255).astype(np.uint8)
        scene.add_geometry(p)
    scene.export(OUTPUT)

if __name__ == '__main__':
    main()
