project('ros_type_introspection', 'cpp', default_options: ['cpp_std=c++14'], version: '0.0.1')

# Designed to produce a minimal binary with no transitive
# binary dependencies (ie. no absl, ROS or boost)

# ROS (rostime headers only)
# Will require ROS paths in pkg-config, usually
# achieved via sourcing one of /opt/ros/*/setup.{zsh,bash,sh}
#
# `export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/opt/ros/*/lib/pkgconfig`
# or similar is also a (less-invasive) option

if get_option('ros_headers_path') == ''
  rostime = dependency('rostime')
  ros_dep = rostime.partial_dependency(includes: true, compile_args: true)
else
  ros_includes = include_directories(get_option('ros_headers_path'))
  ros_dep = declare_dependency(include_directories: ros_includes)
endif

cpp_args = []

# common_cxx
# Fetched via meson wrap
common_cxx = subproject('common_cxx')
lib_dep = common_cxx.get_variable('common_cxx_dep')
cpp_args += '-DENABLE_COMMON_CXX'

# Public includes
includes = include_directories('include')
install_subdir('include/ros_type_introspection', install_dir: 'include')

# Serialisation produces a dependency
if not get_option('serialization')
  cpp_args += '-DDISABLE_SERIALIZATION'
endif

lib_sources = ['src/ros_type.cpp', 'src/ros_field.cpp', 'src/stringtree_leaf.cpp', 'src/ros_message.cpp', 'src/substitution_rule.cpp', 'src/ros_introspection.cpp']
ros_type_introspection_lib = library('ros_type_introspection',
                                     lib_sources,
                                     dependencies: [ros_dep, lib_dep],
                                     include_directories: [includes, ros_includes],
                                     cpp_args: cpp_args,
                                     install: true)

ros_type_introspection_dep = declare_dependency(link_with: [ros_type_introspection_lib], dependencies: [ros_dep, lib_dep], include_directories: [includes, ros_includes], compile_args: cpp_args)
meson.override_dependency('ros_type_introspection', ros_type_introspection_dep)
