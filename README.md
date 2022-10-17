# tmit-star

This repo provides an implementation of TMIT*, a practical almost-surely asymptotically optimal integrated task and motion planner, as initially presented in ["Task and Motion Informed Trees (TMIT*): Almost-Surely Asymptotically Optimal Integrated Task and Motion Planning"](https://ieeexplore.ieee.org/document/9869707/).

If you use this software in your research, please cite the following publication:

```bibtex
@article{thomason_tmit_star_2022,
  author={Thomason, Wil and Strub, Marlin P. and Gammell, Jonathan D.},
  journal={IEEE Robotics and Automation Letters}, 
  title={Task and Motion Informed Trees (TMIT*): Almost-Surely Asymptotically Optimal Integrated Task and Motion Planning}, 
  year={2022},
  volume={7},
  number={4},
  pages={11370-11377},
  doi={10.1109/LRA.2022.3199676}
}
```
## Building

You'll need to manually install the following dependencies (also listed in `meson.build`):
- The [Meson](https://mesonbuild.com/) build system
- [CMake](https://cmake.org/)
- A modern C++ compiler (we use C++20 features; `clang++ 14.0.6` is known to work)
- [Boost](https://www.boost.org/)
- [Glew](https://glew.sourceforge.net/) (for debug visualizer)
- [GLFW3](https://www.glfw.org/) (for debug visualizer)
- [GLM](https://github.com/g-truc/glm) (for debug visualizer)
- [LuaJIT](https://luajit.org/)
- [NLOPT](https://github.com/stevengj/nlopt)
- [Z3](https://github.com/Z3Prover/z3)

All other dependencies are automatically downloaded and configured in `subprojects/` using Meson's Wrap system.

Once all dependencies are installed, run the `./build.sh` script with `zsh`.
For the basic optimized build, use `./build.sh release --lto`; the script also supports debug, sanitizer, and PGO builds (see the script source for relevant args).

This will result in a binary `build/exoplanet`.

## Usage

You can run the planner directly, but the `run.sh` script provides a more convenient interface.
For basic usage, run `./run.sh {PATH_TO_PROBLEM_SPECIFICATION} {ADDITIONAL PLANNER ARGS}`.
The script also supports running with `perf record`, `lldb`, and `rr` for debugging.
To see a listing of planner args, run `./run.sh --help`.
Example problem specifications live in the `problems/` directory - try `./run.sh problems/clutter/problems/clutter_pb_3.json` for a basic example.
Example Lua predicate implementations are provides in `lua/predicates.lua` - you can easily tailor these to your needs.
PDDL files, robot, and object/obstacle geometry models for the example problems are provided in the `problems` and `models` subdirectories.

## Notice

This software is provided as-is, with no guarantees of correctness, continued maintenance, etc.
Feel free to file an issue/PR if you find a problem or would like to contribute.
