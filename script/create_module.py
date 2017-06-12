from __future__ import print_function

#import shutil
import getopt
import sys
import errno    
import os  

root_dir="."
parent_name="visp_contrib"
module_name="contrib"
class_name="vpContrib"
visp_modules="visp_io visp_gui visp_blob visp_vs visp_mbt visp_tt_mi visp_detection visp_robot visp_sensor"

platform_type = sys.platform
def convertPathToWin32(path_):
  return path_.replace("/", "\\")

def usage():
  print(" ")
  print("Create a ViSP module source tree for contrib:")
  print(" ")
  print("<root directory>")
  print("|-- <parent name>")
  print("    |-- modules")
  print("        |-- <module name>")
  print("            |-- CMakeLists.txt")
  print("            |-- include")
  print("            |   |-- visp3")
  print("            |       |-- <module name>")
  print("            |           |-- <class name>.h")
  print("            |-- src")
  print("                |-- <class name>.cpp")
  print(" ")
  print("Usage:")
  print("  python", sys.argv[0], "[--root-dir=<root directory>] [--parent-name=<parent name>] [--module-name=<module name>] [--class-name=<class name>] [--visp-modules=<visp modules deps>] [--help]")
  print(" ")
  print("Options:")
  print("  --root-dir=<root directory>, -r <root directory>")
  print("      Default: \"", root_dir, "\"", sep='')
  print("      Root location of the module tree.")
  print(" ")
  print("  --parent-name=<parent name>, -p <parent name>")
  print("      Default: \"", parent_name, "\"", sep='')
  print("      Optional parent folder that allows to give a name to a collection of contrib modules.")
  print(" ")
  print("  --module-name=<module name>, -m <module name>")
  print("      Default: \"", module_name, "\"", sep='')
  print("      Name of the module to create as a new ViSP module.")
  print(" ")
  print("  --class-name=<module name>, -c <module name>")
  print("      Default: \"", class_name, "\"", sep='')
  print("      From this class name corresponding header and source files are created.")
  print(" ")
  print("  --visp-modules=<visp modules deps>, -v <visp modules deps>")
  print("      Default: \"", visp_modules, "\"", sep='')
  print("      ViSP modules dependencies: visp_core, ...")
  print(" ")
  print("  --help, -h")
  print("      Print this help.")
  return

def mkdir_p(path_):
  try:
    os.makedirs(path_)
  except OSError as exc:  # Python >2.5
    if exc.errno == errno.EEXIST and os.path.isdir(path_):
      pass
    else:
      raise
          
def create_tree(root_dir_, parent_name_, module_name_):
  path_ = root_dir_ + "/" + parent_name_ + "/modules/" + module_name_
  path_src_ = path_ + "/src"
  path_inc_ = path_ + "/include/visp3/" + module_name_
  path_test_ = path_ + "/test"
  print("Create folder:", path_src_)
  print("Create folder:", path_inc_)
  print("Create folder:", path_test_)
  mkdir_p(path_src_)
  mkdir_p(path_inc_)
  mkdir_p(path_test_)
  return

def create_cmakelists(root_dir_, parent_name_, module_name_, visp_modules_):
  path_ = root_dir_ + "/" + parent_name_ + "/modules/" + module_name_
  filename_ = path_ + "/CMakeLists.txt"
  print("Create file  :", filename_)
  file_ = open(filename_, "w") 
  file_.write("vp_add_module(" + module_name_ + " " + visp_modules_ + ")\n")
  file_.write("vp_glob_module_sources()\n")
  file_.write("vp_module_include_directories()\n")
  file_.write("vp_create_module()\n\n")
  file_.write("vp_add_tests()\n")
  file_.close()
  return

def create_header(root_dir_, parent_name_, module_name_, class_name_):
  path_ = root_dir_ + "/" + parent_name_ + "/modules/" + module_name_ + "/include/visp3/" + module_name_ 
  filename_ = path_ + "/" + class_name_ + ".h"
  print("Create file  :", filename_)
  file_ = open(filename_, "w") 
  file_.write("#ifndef __" + class_name_ + "_h__\n")
  file_.write("#define __" + class_name_ + "_h__\n\n")
  file_.write("#include <visp3/core/vpConfig.h>\n\n")
  file_.write("class VISP_EXPORT " + class_name_ + "\n")
  file_.write("{\n")
  file_.write("public:\n")
  file_.write("  " + class_name_ + "();\n")
  file_.write("  virtual ~" + class_name_ + "(){};\n")
  file_.write("};\n\n")
  file_.write("#endif\n")
  file_.close()
  return

def create_source(root_dir_, parent_name_, module_name_, class_name_):
  path_ = root_dir_ + "/" + parent_name_ + "/modules/" + module_name_ + "/src/"
  filename_ = path_ + class_name_ + ".cpp"
  print("Create file  :", filename_)
  file_ = open(filename_, "w") 
  file_.write("#include <iostream>\n\n")
  file_.write("#include <visp3/" + module_name_+ "/" + class_name_ + ".h>\n\n")
  file_.write(class_name_ + "::" + class_name + "()\n")
  file_.write("{\n")
  file_.write("  std::cout << \"I'm in my first contrib module\" << std::endl;\n")
  file_.write("}\n")
  file_.close()
  return

def create_test(root_dir_, parent_name_, module_name_, class_name_):
  path_ = root_dir_ + "/" + parent_name_ + "/modules/" + module_name_ + "/test/"
  filename_ = path_ + "test-" + class_name_ + ".cpp"
  print("Create file  :", filename_)
  file_ = open(filename_, "w") 
  file_.write("#include <visp3/" + module_name_+ "/" + class_name_ + ".h>\n\n")
  file_.write("int main()\n")
  file_.write("{\n")
  file_.write("  " + class_name_ + " contrib;\n")
  file_.write("}\n")
  file_.close()
  return

def print_howto(root_dir_, parent_name_, module_name_, class_name_):
  path_ = root_dir_ + "/" + parent_name_
  if platform_type == 'win32':
    convertPathToWin32(path_)
  print("\nThe new module \"" + module_name_ + "\" is available in : " + path_, sep='')
  print("\nTo build this new module, launch the following instructions:")
  print("- get visp source from github")
  if platform_type == 'win32':
    root_dir_ = convertPathToWin32(root_dir_)
  print("  $ cd ", root_dir_, sep='')
  print("  $ git clone https://github.com/lagadic/visp.git")
  print("- create a build folder")
  path_build_ = root_dir_ + "/" + parent_name_ + "-build"
  if platform_type == 'win32':
    path_build_ = convertPathToWin32(path_build_)
  
  print("  $ mkdir ", path_build_, sep='')
  print("  $ cd ", path_build_, sep='')
  print("- configure the build")
  if platform_type == 'win32':
    print("  $ cmake-gui ../visp")
    print("  -> Click \"Configure\" button")
    print("  -> Secify the generator for this projet and click \"Finish\" button")
    print("  -> Set VISP_CONTRIB_MODULES_PATH to ", path_ )
    print("  -> Click \"Configure\" button")
    print("  -> Click \"Generate\" button")
  else:
    print("  $ cmake ../visp -DVISP_CONTRIB_MODULES_PATH=", root_dir_ + "/" + parent_name_, sep='')

  print("- build", module_name_, "module and the corresponding test")
  if platform_type == 'win32':
    print("  -> Open ", path_build_, "\\visp.sln solution file and build the project", sep='')
  else:
    print("  $ make -j4 visp_", module_name_, sep='')
    print("  $ make test-", class_name_, sep='')
    print("- execute the corresponding test")
    print("  $ ./modules/" + module_name_ + "/test-", class_name_, sep='')
    print("- build all the project")
    print("  $ make -j4 ")
  return


def create_module(root_dir_, parent_name_, module_name_, class_name_, visp_modules_):
  create_tree(root_dir_, parent_name_, module_name_)
  create_cmakelists(root_dir_, parent_name_, module_name_, visp_modules_)
  create_header(root_dir_, parent_name_, module_name_, class_name_)
  create_source(root_dir_, parent_name_, module_name_, class_name_)
  create_test(root_dir_, parent_name_, module_name_, class_name_)
  print_howto(root_dir_, parent_name_, module_name_, class_name_)

try:
  opts, args = getopt.getopt(sys.argv[1:],"c:hm:p:r:v:", ["root-dir=", "parent-name=", "module-name=", "class-name=", "help"])

except getopt.GetoptError:
  # print help information and exit:
  usage()
  print("Error: ")
  print("  Bad usage. ")
  sys.exit(2)

for o, v in opts:
  if o in ("--root-dir", "-r"):
    root_dir = v
  if o in ("--parent-name", "-p"):
    parent_name = v
  if o in ("--module-name", "-m"):
    module_name = v
  if o in ("--class-name", "-c"):
    class_name = v
  if o in ("--visp-modules", "-v"):
    visp_modules = v
  if o in ("--help", "-h"):
    usage()
    sys.exit(2)

print("New \"", module_name, "\" module creation in process...", sep='')

create_module(root_dir, parent_name, module_name, class_name, visp_modules)
