# Import a backend, we use torch in this example.
import gpytorch
import torch

# Import the geometric_kernels backend.
import geometric_kernels
import geometric_kernels.torch

# Import the Mesh space and the general-purpose MaternGeometricKernel
from geometric_kernels.spaces.mesh import Mesh
from geometric_kernels.kernels import MaternGeometricKernel

# The GPyTorch frontend of GeometricKernels
from geometric_kernels.frontends.gpytorch import GPyTorchGeometricKernel

# Sampling routines we will use to create a dummy dataset
from geometric_kernels.kernels import default_feature_map
from geometric_kernels.sampling import sampler
from geometric_kernels.utils.utils import make_deterministic

# Stuff
import numpy as np
#import optax
import plotly.graph_objects as go
from plotly.subplots import make_subplots
from pathlib import Path

def update_figure(fig):
    """Utility to clean up figure"""
    fig.update_layout(scene_aspectmode="cube")
    fig.update_scenes(xaxis_visible=False, yaxis_visible=False, zaxis_visible=False)
    # fig.update_traces(showscale=False, hoverinfo="none")
    fig.update_layout(margin=dict(l=0, r=0, t=0, b=0))

    fig.update_layout(plot_bgcolor="rgba(0,0,0,0)", paper_bgcolor="rgba(0,0,0,0)")
    fig.update_layout(
        scene=dict(
            xaxis=dict(showbackground=False, showticklabels=False, visible=False),
            yaxis=dict(showbackground=False, showticklabels=False, visible=False),
            zaxis=dict(showbackground=False, showticklabels=False, visible=False),
        )
    )
    return fig

def plot_mesh(mesh: Mesh, vertices_colors = None, **kwargs):
    plot = go.Mesh3d(
        x=mesh.vertices[:, 0],
        y=mesh.vertices[:, 1],
        z=mesh.vertices[:, 2],
        i=mesh.faces[:, 0],
        j=mesh.faces[:, 1],
        k=mesh.faces[:, 2],
        intensity=vertices_colors,
        **kwargs
    )
    return plot


def main():
    mesh = Mesh.load_mesh(str(Path.cwd().parent / "teddy.obj"))
    print("Number of vertices in the mesh:", mesh.num_vertices)
    # # Define the camera
    # camera = dict(
    #     up=dict(x=0, y=1, z=0),
    #     center=dict(x=0, y=0, z=0),
    #     eye=dict(x=0, y=0.7, z=1.25)
    # )

    # plot = plot_mesh(mesh)
    # fig = go.Figure(plot)
    # update_figure(fig)
    # fig.update_layout(
    #     scene_camera=camera
    # )
    # fig.show("png")
    num_data = 50
    key = torch.Generator()
    key.manual_seed(1234)

    xs_train = torch.randint(low=0, high=mesh.num_vertices, size=(num_data, 1), generator=key, dtype=torch.int64)
    xs_test = torch.arange(mesh.num_vertices, dtype=torch.int64)[:, None]
    # print("xs_train:", xs_train)
    # print("xs_test:", xs_test)
    base_kernel = MaternGeometricKernel(mesh)

    params = base_kernel.init_params()
    params["lengthscale"] = torch.tensor([5.0], dtype=torch.float64)
    params["nu"]  = torch.tensor([2.5], dtype=torch.float64)

    feature_map = default_feature_map(kernel=base_kernel)
    sample_paths = make_deterministic(sampler(feature_map), key)

    _, ys_train  = sample_paths(xs_train, params)
    key, ys_test = sample_paths(xs_test,  params)
    ys_train = ys_train[:, 0]
    ys_test = ys_test[:, 0]

    assert(torch.allclose((ys_test[xs_train[:, 0]]), ys_train))

if __name__ == "__main__":
    main()