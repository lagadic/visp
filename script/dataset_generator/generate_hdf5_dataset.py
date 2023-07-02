import blenderproc as bproc
'''
Script to generate a synthetic dataset, stored in hdf5 format.
This dataset can contain:
 - RGB images
 - Object poses
 - Segmentation maps
 - Bounding boxes
 - Depth map
 - Normal map

To run, blenderproc and other things should be installed (virtual environment recommended):
$ conda activate dataset_generation
$ pip install blenderproc numpy
$ blenderproc quickstart # verify that it works, install minor dependencies


'''

'''
TODO:
 - Random objects selection: max number of objects, location, rotation, sample_on_surface tuto
 - Random distractors
 - Random : color and orientation for spot
'''
import os
import numpy as np
import argparse
import json
from operator import itemgetter
from pathlib import Path
from typing import Callable, Dict, List

from blenderproc.python.types import MeshObjectUtility, EntityUtility

def homogeneous_inverse(aTb):
  bTa = aTb.copy()
  bTa[:3, :3] = bTa[:3, :3].T
  bTa[:3, 3] = -bTa[:3, :3] @ bTa[:3, 3]
  return bTa
def homogeneous_no_scaling(object: bproc.types.MeshObject):
  worldTlocal = np.eye(4)
  worldTlocal[:3, :3] = object.get_rotation_mat()
  worldTlocal[:3, 3] = object.get_location()
  return worldTlocal

def point_in_bounding_box(object: bproc.types.MeshObject, bb_scale=1.0):
  '''
  Sample a random point in the bounding box of an object.
  the bounding box can be scaled with bb_scale if we wish to sample a point "near" the true bounding box
  Returned coordinates are in world frame
  '''
  
  bb = object.get_bound_box() * bb_scale # 8 x 3
  worldTlocal = homogeneous_no_scaling(object) # local2world includes scaling
  localTworld = homogeneous_inverse(worldTlocal)
  # go to object space to sample with axis aligned bb, since sampling in world space would be harder
  bb_local = localTworld @ np.concatenate((bb, np.ones((8, 1))), axis=-1).T
  bb_local = bb_local.T[:, :3] # 8 X 3
  mins, maxes = np.min(bb_local, axis=0), np.max(bb_local, axis=0)
  point_local = np.zeros(3)
  for i in range(3):
    point_local[i] = np.random.uniform(mins[i], maxes[i])
  point = worldTlocal @ np.concatenate((point_local, np.ones(1)))[:, None]
  point = point[:3, 0]
  return point

  


def randomize_pbr(objects):
  # set shading and physics properties and randomize PBR materials
  for j, obj in enumerate(objects):
      obj.set_shading_mode('auto')
      mat = obj.get_materials()[0]       
      mat.set_principled_shader_value("Roughness", np.random.uniform(0, 1.0))
      mat.set_principled_shader_value("Specular", np.random.uniform(0, 1.0))
      mat.set_principled_shader_value("Metallic", np.random.uniform(0, 1.0))
      
def add_displacement(objects, max_displacement_strength=0.05):
  for obj in objects:
    obj.add_uv_mapping("cylinder")

    # Create a random procedural texture
    noise_models = ["CLOUDS", "DISTORTED_NOISE", "MAGIC", "MARBLE",
                     "MUSGRAVE", "NOISE", "STUCCI", "VORONOI", "WOOD"]
    texture = bproc.material.create_procedural_texture(noise_models[np.random.choice(len(noise_models))])
    # Displace the vertices of the object based on that random texture
    obj.add_displace_modifier(
        texture=texture,
        strength=np.random.uniform(0, max_displacement_strength),
        subdiv_level=np.random.randint(1, 3),
    )


class Generator:
  def __init__(self, config_path):
    self.config_path = config_path
    with open(self.config_path, 'r') as json_config_file:
      self.json_config = json.load(json_config_file)
    print(self.json_config)
    os.environ['BLENDER_PROC_RANDOM_SEED'] = self.json_config['blenderproc_seed']

  def init(self):
    np.random.seed(self.json_config['numpy_seed'])
    self.objects, self.classes = self.load_objects()
    self.cc_textures = bproc.loader.load_ccmaterials(self.json_config['cc_textures_path'])

  def load_objects(self) -> Dict[str, bproc.types.MeshObject]:
    '''
    Load the objects in the models directory
    The directory should have the following structure:
    - models/
    --- obj_1_name/
    ----- model.obj
    --- obj_2_name/
    ----- model.obj
    Returns a dict where keys are the model names 
    (from the containing folder name of each object, 
    in the example case: 'obj_1_name' and 'obj_2_name')
    and the values are the loaded model
    '''
    models_dict = {}
    class_dict = {}
    models_path = Path(self.json_config['models_path']).absolute()
    cls = 1
    assert models_path.exists() and models_path.is_dir(), f'Models path {models_path} must exist and be a directory'
    for model_dir in models_path.iterdir():
      if not model_dir.is_dir():
        continue
      model_name = model_dir.name
      for content in model_dir.iterdir():
        if not content.is_file():
          continue
        load_fn: Callable[[str], List[bproc.types.MeshObject]] = None
        if content.name.endswith('.obj') or content.name.endswith('.ply'):
          load_fn = bproc.loader.load_obj
        if load_fn is not None:
          print(models_dict.keys())
          assert model_name not in models_dict, f'A folder should contain a single object, but {content} contains multiple objects (.blend or .obj)'
          models = load_fn(str(content.absolute()))
          assert len(models) > 0, f'Loaded an empty file: {content}'
          model = models[0]
          if len(models) > 1:
            model.join_with_other_objects(models[1:])
          model.set_cp('category_id', cls)
          model.hide() # Hide by default

          models_dict[model_name] = model
          class_dict[model_name] = cls
          cls += 1

    return models_dict, class_dict

  def render(self) -> Dict:
    '''
    Setup rendering: enable depth, normal and segmentation output
    '''
    depth, normals, segmentation = itemgetter('depth', 'normals', 'segmentation')(self.json_config['dataset'])
    bproc.renderer.set_max_amount_of_samples(self.json_config['rendering']['max_num_samples'])
    if depth:
      bproc.renderer.enable_depth_output(activate_antialiasing=False)
    if normals:
      bproc.renderer.enable_normals_output()
    # if segmentation:
    #   bproc.renderer.enable_segmentation_output(map_by=['category_id', 'instance_id'], default_values={'category_id': None, 'instance_id': None})
    data = bproc.renderer.render()
    if segmentation:
      data.update(bproc.renderer.render_segmap(map_by=["instance", "class"]))
    return data
  
  def save_data(self, objects: List[bproc.types.MeshObject], data: Dict):
    pass

  def set_camera_intrinsics(self) -> None:
    '''
    Set camera intrinsics from config.
    Randomized depending on randomize_params_percent. This does not impact image resolution
    '''
    px, py, u0, v0, h, w , r = itemgetter('px', 'py', 'u0', 'v0', 'h', 'w', 'randomize_params_percent')(self.json_config['camera'])
    r = r / 100.0
    randomize = lambda x: x + np.random.uniform(-x * r, x * r)
    K = [
      [randomize(px), 0, randomize(u0)],
      [0, randomize(py), randomize(v0)],
      [0, 0, 1],
    ]
    bproc.camera.set_intrinsics_from_K_matrix(K, w, h)


  def create_distractors(self, scene_size, other_objects) -> List[bproc.types.MeshObject]:
    json_distractors = self.json_config['scene']['distractors']
    min_count, max_count = itemgetter('min_count', 'max_count')(json_distractors)
    min_size, max_size = itemgetter('min_size_rel_scene', 'max_size_rel_scene')(json_distractors)
    displacement_strength = itemgetter('displacement_max_amount')(json_distractors)
        
    count = np.random.randint(min_count, max_count + 1)
    def sample_pose(obj: bproc.types.MeshObject):
      loc = np.random.uniform([-scene_size / 2, -scene_size / 2, -scene_size / 2], [scene_size / 2, scene_size / 2, scene_size / 2])
      obj.set_location(loc)
      obj.set_scale(np.random.uniform(scene_size * min_size, scene_size * max_size, size=3))
      obj.set_rotation_euler(bproc.sampler.uniformSO3())
    distractor_type = np.random.choice(['CUBE', 'CYLINDER', 'CONE', 'PLANE', 'SPHERE', 'MONKEY'], size=count, replace=True)
    distractors = [bproc.object.create_primitive(distractor_type[c]) for c in range(count)]
    
    bproc.object.sample_poses(
      distractors,
      sample_pose_func=sample_pose,
      objects_to_check_collisions=None
    )

    for obj in distractors:
      random_texture = np.random.choice(self.cc_textures)
      obj.replace_materials(random_texture)
    randomize_pbr(distractors)
    add_displacement(distractors, displacement_strength)
    return distractors
    
  def create_lights(self, scene_size, count, target_objects) -> List[bproc.types.Light]:
    light_json = self.json_config['scene']['lights']
    min_count, max_count = itemgetter('min_count', 'max_count')(light_json)
    min_intensity, max_intensity = itemgetter('min_intensity', 'max_intensity')(light_json)

    count = np.random.randint(min_count, max_count + 1)
    
    point_light_count = np.sum(np.random.choice(2, size=count, replace=True))
    spot_light_count = count - point_light_count
    
    lights = []
    for l in range(point_light_count):
      light = bproc.types.Light('POINT')
      loc = np.random.uniform([-scene_size / 2, -scene_size / 2, -scene_size / 2], [scene_size / 2, scene_size / 2, scene_size / 2])
      light.set_location(loc)
      light.set_energy(np.random.uniform(min_intensity, max_intensity))
      light.set_color(np.random.uniform(0.5, 1.0, size=3))
      lights.append(light)
    for l in range(spot_light_count):
      light = bproc.types.Light('SPOT')
      loc = np.random.uniform([-scene_size / 2, -scene_size / 2, -scene_size / 2], [scene_size / 2, scene_size / 2, scene_size / 2])
      light.set_location(loc)
      light.set_energy(np.random.uniform(min_intensity, max_intensity))
      light.set_color(np.random.uniform(0.5, 1.0, size=3))

      looked_at_obj = np.random.choice(len(target_objects))
      looked_at_obj = target_objects[looked_at_obj]
      poi = point_in_bounding_box(looked_at_obj)
      R = bproc.camera.rotation_from_forward_vec(poi - light.get_location())
      light.set_rotation_mat(R)
      lights.append(light)
    return lights
  
  def create_target_objects(self) -> List[bproc.types.MeshObject]:
    json_objects = self.json_config['scene']['objects']
    min_count, max_count, replace = itemgetter('min_count', 'max_count', 'multiple_occurences')(json_objects)
    scale_noise, displacement_amount, pbr_noise = itemgetter('scale_noise', 'displacement_max_amount', 'pbr_noise')(json_objects)
    
    object_keys = list(self.objects.keys())

    if not replace:
      max_count = min(max_count, len(object_keys))
    count = np.random.randint(min_count, max_count + 1)
    selected_key_indices = np.random.choice(len(object_keys), size=count, replace=replace)
    
    objects: List[bproc.types.MeshObject] = [self.objects[object_keys[index]].duplicate() for index in selected_key_indices]
    for object in objects:
      object.hide(False)
      if scale_noise > 0.0:
        random_scale = np.random.uniform(-scale_noise, scale_noise) + 1.0
        object.set_scale([random_scale, random_scale, random_scale]) # Uniform scaling

    if displacement_amount > 0.0:
      add_displacement(objects, displacement_amount)
    if pbr_noise:
      randomize_pbr(objects)
    
    return objects

  def create_scene(self):
    '''
    Create a basic scene, a square room.
    The size of the room is dependent on the size of the biggest object multiplied by a user supplied param
    Each wall has a random texture, sampled from cc0 materials
    Distractors are added randomly in the room
    Random lights are also placed
    '''

    room_size = 0.0
    room_size_multiplier = self.json_config['scene']['room_size_multiplier']
    
    assert room_size_multiplier >= 1.0, 'Room size multiplier should be more than one'
    objects = self.create_target_objects()

    for object in objects:
        print()
        bb = object.get_bound_box()
        print(object.get_name(), object.get_scale(), bb)
        for corner_index in range(len(bb) - 1):
          dists = np.linalg.norm(bb[corner_index+1:] - bb[corner_index], axis=-1, ord=2)
          print(dists)
          room_size = max(np.max(dists), room_size)
          print(object.get_location())
    print('room size = ', room_size)
    size = room_size * room_size_multiplier

    ground = bproc.object.create_primitive('PLANE')
    ground.set_location([0, 0, -size / 2])
    ground.set_scale([size / 2, size / 2, 1])
    room_objects = [ground]
    wall_data = [
      {'loc': [size / 2, 0, 0], 'rot': [0, np.pi / 2, 0]},
      {'loc': [-size / 2, 0, 0], 'rot': [0, np.pi / 2, 0]},
      {'loc': [0, size / 2, 0], 'rot': [np.pi / 2, 0, 0]},
      {'loc': [0, -size / 2, 0], 'rot': [np.pi / 2, 0, 0]},
    ]
    for w_data in wall_data:
      wall = bproc.object.create_primitive('PLANE')
      wall.set_location(w_data['loc'])
      wall.set_rotation_euler(w_data['rot'])
      wall.set_scale([size / 2, size / 2, 1])
      room_objects.append(wall)

    for obj in room_objects:
      random_texture = np.random.choice(self.cc_textures)
      obj.replace_materials(random_texture)
    randomize_pbr(room_objects)

    def sample_pose(obj: bproc.types.MeshObject):
      loc = np.random.uniform([-size / 2, -size / 2, -size / 2], [size / 2, size / 2, size / 2])
      obj.set_location(loc)
      obj.set_rotation_euler(bproc.sampler.uniformSO3())
    bproc.object.sample_poses(
      objects,
      sample_pose_func=sample_pose,
      objects_to_check_collisions=None
    )

    distractors = self.create_distractors(size, room_objects + objects) # TODO: place real objects
    lights = self.create_lights(size, 5, objects)
    return objects, room_objects, distractors, lights

# from blenderproc.python.renderer import RendererUtility

if __name__ == '__main__':

  parser = argparse.ArgumentParser()
  parser.add_argument('--config', required=True, type=str, help='Path to the JSON configuration file for the dataset generation script')
  args = parser.parse_args()

  # RendererUtility.set_render_devices(use_only_cpu=True)
  # bproc.clean_up(clean_up_camera=True)
  generator = Generator(args.config)
  bproc.init() # Works if you have a GPU
  generator.init()
  generator.set_camera_intrinsics()
  # Create a simple object:
  obj = bproc.object.create_primitive("MONKEY")
  obj.set_scale([0.01, 0.01, 0.01])
  add_displacement([obj])


  # Set the camera to be in front of the object
  poi = bproc.object.compute_poi([obj])
  location = np.random.uniform([-1, -1, -1], [1,1,1])
  rotation_matrix = bproc.camera.rotation_from_forward_vec(poi - location, inplane_rot=np.random.uniform(-0.7854, 0.7854))
  cam2world_matrix = bproc.math.build_transformation_mat(location, rotation_matrix)
  bproc.camera.add_camera_pose(cam2world_matrix)
  generator.create_scene()

  # Render the scene

  data = generator.render()
  print(data)

  #seg_data = bproc.renderer.render_segmap(map_by=["instance", "class", "name"])

  # Write the rendering into an hdf5 file
  bproc.writer.write_hdf5("output/", data)
  # bproc.writer.write_coco_annotations('output/coco',
  #                                   instance_segmaps=seg_data["instance_segmaps"],
  #                                   instance_attribute_maps=seg_data["instance_attribute_maps"],
  #                                   colors=data["colors"],
  #                                   color_file_format="PNG")
  
  
