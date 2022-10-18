project('tmit-star', 'cpp', version : '0.1', default_options: ['warning_level=3', 'cpp_std=c++20'])

# Dependencies
cmake = import('cmake')

autodiff = dependency('autodiff', required: false)
if not autodiff.found()
  autodiff_cmake_opts = cmake.subproject_options()
  autodiff_cmake_opts.add_cmake_defines({'AUTODIFF_BUILD_EXAMPLES': false})
  autodiff_cmake_opts.set_override_option('cpp_std', 'c++20')
  autodiff_cmake = cmake.subproject('autodiff', options: autodiff_cmake_opts)
  autodiff = autodiff_cmake.dependency('autodiff')
endif

# Hack because the current OMPL release isn't new enough
ompl_cmake_opts = cmake.subproject_options()
ompl_cmake_opts.set_override_option('cpp_std', 'c++20')
ompl_cmake = cmake.subproject('ompl', options: ompl_cmake_opts)
ompl = ompl_cmake.dependency('ompl')

dependencies = [
  autodiff,
  dependency('boost'),
  dependency('bullet'),
  dependency('cxxopts'),
  dependency('date'),
  dependency('dbg'),
  dependency('eigen3', fallback: ['eigen', 'eigen3']),
  dependency('filestl'),
  dependency('fmt'),
  dependency('glew'),
  dependency('glfw3'),
  dependency('glm'),
  dependency('lazycsv'),
  dependency('luajit'),
  dependency('nlohmann_json'),
  dependency('nlopt'),
  dependency('openmp'),
  dependency('robin_hood'),
  dependency('sexplib'),
  dependency('spdlog'),
  dependency('tinyobjloader'),
  dependency('urdfdom'),
  dependency('z3'),
  ompl
]

cpp_args = [
  '-Wno-deprecated-enum-enum-conversion',
  '-Wno-deprecated-anon-enum-enum-conversion',
  '-Wno-gnu-zero-variadic-macro-arguments',
  '-fsanitize-recover=all',
  '-march=native'
]

link_args = ['-fsanitize-recover=all']

source_files = [
  'tmit-star.cc',
  'experiments/experiments.cc',
  'input/input.cc',
  'input/pddl.cc',
  'input/scene.cc',
  'itstar/AITstar.cpp',
  'itstar/aitstar/src/ImplicitGraph.cpp',
  'output/output.cc',
  'planner/collisions.cc',
  'planner/cspace.cc',
  'planner/expression_tree.cc',
  'planner/goal.cc',
  'planner/lua_env.cc',
  'planner/mode.cc',
  'planner/mode_atlas.cc',
  'planner/motion_validation.cc',
  'planner/optimization_objective.cc',
  'planner/sampler.cc',
  'planner/scenegraph.cc',
  'planner/solver.cc',
  'planner/symbolic.cc',
  'planner/task_plan.cc',
  'planner/task_plan_propagator.cc',
  'util/utils.cc',
]

# Main executable
tmit_star = executable('tmit-star',
  source_files,
  include_directories: ['experiments', 'input', 'output', 'planner', 'util', 'itstar'],
  dependencies: dependencies,
  cpp_args: cpp_args,
  link_args: link_args
)
