from typing import List, Optional, Callable
import numpy as np
import json
from matplotlib import pyplot as plt
import argparse
from pathlib import Path

# Use latex in plot legends and labels
plt.rc('text', usetex=True)
plt.rc('text.latex', preamble=r'\usepackage{amsmath}')

def basic_plot(data: np.ndarray, title: Optional[str] = None, legend: List[str] = None, ylabel: str = None, fontsize=16, titlefontsize=24,
              before_fn: Optional[Callable[[plt.Figure, np.ndarray], None]] = None,
              after_fn: Optional[Callable[[plt.Figure, np.ndarray], None]] = None) -> plt.Figure:
  fig = plt.figure()
  if before_fn is not None:
      before_fn(fig, data)

  plt.plot(data)
  if legend is not None:
    plt.legend(legend, fontsize=fontsize)
  if ylabel is not None:
    plt.ylabel(ylabel, fontsize=fontsize)
  plt.xlabel('Iterations', fontsize=fontsize)
  plt.grid()
  if title is not None:
    plt.title(title, fontsize=titlefontsize)

  if after_fn is not None:
      after_fn(fig, data)
  return fig

if __name__ == '__main__':
  parser = argparse.ArgumentParser(description='Plot the data related to a basic control law. Input data should be generated with the tutorial located in tutorials/visual-servo/ibvs/tutorial-ibvs-4pts-json')
  parser.add_argument('--data', type=str, help='The JSON file containing the data')
  parser.add_argument('--plot_folder', type=str, help='The folder in which to save the plots')
  args = parser.parse_args()

  json_path = Path(args.data).absolute()
  assert json_path.exists(), f'JSON file {json_path} not found'
  json_data = None
  with open(json_path, 'r') as json_file:
      json_data = json.load(json_file)

  plot_folder_path = Path(args.plot_folder).absolute()
  plot_folder_path.mkdir(exist_ok=True)

  def bounds(_fig, data):
      xmin = 0
      xmax = len(data)
      ymin = min(0, np.min(data))
      ymax = np.max(data)
      plt.xlim((xmin, xmax))
      plt.ylim((ymin, ymax))
  def make_hline(_fig, data):
      plt.axhline(0, 0, len(data), c='k')

  def sequence_fn(*fns) -> Callable:
      def multi(*args):
        for fn in fns:
            fn(*args)
      return multi

  if 'errorNorm' in json_data:
    print('Generating error norm plot...')
    error_norm = np.array(json_data['errorNorm'])
    fig_error = basic_plot(error_norm, legend=None,
                          title='Visual error norm',
                          ylabel=r'$\lVert\mathbf{e}\rVert^2_2$',
                          before_fn=sequence_fn(bounds, make_hline))
    plt.tight_layout()
    plt.savefig(plot_folder_path / 'error.pdf')
    plt.close()

  if 'velocities' in json_data:
    print('Generating velocity plot...')
    velocities = np.array([o['data'] for o in json_data['velocities']])
    velocities_legend = [fr'$\mathbf{{\upsilon_{axis}}}$' for axis in ['x', 'y', 'z']] + \
    [fr'$\mathbf{{\omega_{axis}}}$' for axis in ['x', 'y', 'z']]
    fig_velocities = basic_plot(velocities, legend=velocities_legend,
                          title='Camera velocity',
                          ylabel=r'$\mathbf{v}$',
                          before_fn=sequence_fn(bounds, make_hline))
    plt.tight_layout()
    plt.savefig(plot_folder_path / 'velocity.pdf')
    plt.close()

  if 'trajectory' in json_data:
    print('Generating 2D and 3D trajectory plots...')
    poses = np.array([o['data'] for o in json_data['trajectory']])
    cdMo = np.array(json_data['parameters']['cdMo']['data']).reshape(4, 4)
    poses_legend = [fr'$\mathbf{{t_{axis}}}$' for axis in ['x', 'y', 'z']] + \
    [fr'$\theta\mathbf{{u_{axis}}}$' for axis in ['x', 'y', 'z']]
    fig_poses = basic_plot(poses, legend=poses_legend,
                          title='Camera pose',
                          ylabel=r'$\mathbf{r}$',
                          before_fn=bounds)
    plt.tight_layout()
    plt.savefig(plot_folder_path / 'cMo.pdf')
    plt.close()

    ax = plt.figure().add_subplot(projection='3d')
    ax.scatter(poses[0, 0], poses[0, 1], poses[0, 2], marker='x', c='r', label='Initial position')
    ax.scatter(cdMo[0, -1], cdMo[1, -1], cdMo[2, -1], marker='x', c='g', label='Desired position')
    ax.plot(poses[:, 0], poses[:, 1], zs=poses[:, 2], label='Camera trajectory')
    plt.title('Camera trajectory in world space', fontsize=24)
    plt.legend()
    plt.tight_layout()
    plt.savefig(plot_folder_path / 'cMo_3d.pdf')
    plt.close()

  if 'features' in json_data and 'desiredFeatures' in json_data:
    print('Generating features plot...')
    desired_features_2d = np.array([[f['x'], f['y']] for f in json_data['desiredFeatures']])
    fig = plt.figure()
    plt.scatter(desired_features_2d[:, 0], desired_features_2d[:, 1], marker='x')
    trajectories_2d = []
    point_count = len(desired_features_2d)
    for i in range(point_count): # Each iteration we have a set of point_count points, iterate on every point to get the 2d trajectory of each point
       trajectories_2d.append(np.array([[points[i]['x'], points[i]['y']] for points in json_data['features']]))
    for trajectory_2d in trajectories_2d:
       plt.plot(trajectory_2d[:, 0], trajectory_2d[:, 1])
    plt.title('IBVS feature trajectories')
    plt.tight_layout()
    plt.savefig(plot_folder_path / 'trajectories_2d.pdf')
    plt.close()

  print(f'Plots were saved to {plot_folder_path}')
