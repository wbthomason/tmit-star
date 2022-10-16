#!/bin/env zsh

prefix=()
suffix=()
profile_run=false
if [ "$1" = "profile" ]; then
  profile_run=true
  prefix+=(perf record --call-graph dwarf -F 99 --)
  shift 1
  TRAPINT() {
    echo "Caught interrupt! Continuing to generate perf file..."
    return 0
  }
elif [ "$1" = "debug" ]; then
  prefix+=(lldb)
  suffix+=(--)
  shift 1
elif [ "$1" = "rr" ]; then
  prefix+=(rr record)
  shift 1
fi

cmd=("${prefix[@]}" build/exoplanet "${suffix[@]}" "${@[@]}")
env "${cmd[@]}"

if $profile_run; then
  perf script -F +pid > exoplanet.perf
fi
