#!/bin/env zsh

# Ensure the build directory exists
if [ ! -d build ]; then
	echo "Creating initial build directory"
	mkdir build
  # NOTE: Try building with g++ and compare performance
	CXX=clang++ CXX_LD=lld meson setup build
fi

# Handle args
local -A opts
zparseopts -A opts -E -D -F - -sanitize:: -lto=use_lto -pgo:: || exit 1
buildtype=$1

# Reconfigure the build if requested
meson_args=()
if [[ "$#opts" -gt 0 || -n "${use_lto}" ]]; then
  if [ -n "${opts[(ie)--sanitize]}" ]; then
    meson_args+=("-Db_sanitize=${opts[--sanitize]}")
    meson_args+=(-Db_lundef=false)
	else
		meson_args+=(-Db_sanitize=none)
	fi

	if [ -n "${use_lto}" ]; then
		meson_args+=(-Db_lto=true)
	else
		meson_args+=(-Db_lto=false)
	fi

  if [ -n "${opts[(ie)--pgo]}" ]; then
		meson_args+=("-Db_pgo=${opts[--pgo]}")
	else
		meson_args+=(-Db_pgo=off)
	fi
fi

if [ -n "$buildtype" ]; then
	meson_args+=("--buildtype=$buildtype")
fi

if [ "${#meson_args}" -gt 0 ]; then
	echo "Reconfiguring build: ${meson_args[@]}"
	meson configure build "${meson_args[@]}" -Db_ndebug=if-release
fi

# Compile
meson compile -C build
