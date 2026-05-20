# Building uno_mex

uno_mex links against numeric libraries (HiGHS, METIS, MUMPS, …) via **CMake CONFIG packages** installed from sibling repos, compiles **official Uno sources** in-tree (`cmake/uno_build`), and builds **MATLAB MEX** targets (`uno_mex`, `uno_options`).

There is **no** separate `cmake --install` step for Uno in the normal workflow.

## Repository layout

CI and local superbuilds expect sibling trees under a common root (`UNO_DEPS_ROOT`):

```text
${UNO_DEPS_ROOT}/
  uno_mex/          ← this repo
  Uno/              ← official sources (FetchContent or checkout)
  HiGHS/
  METIS/
  GKlib/
  bqpd_lib/
  coinhsl/
  MUMPS_cmake/
  OpenBLAS/
  MexUtilities/     ← separate from Uno numeric stack; required for MEX
```

Each dependency is configured with the **same CMake preset name**, installed to:

```text
<repo>/out/install/<preset-name>/
```

uno_mex discovers them via [`cmake/UnoDeps.cmake`](cmake/UnoDeps.cmake). The preset name defaults to the leaf of `uno_mex/out/build/<preset-name>` (i.e. `UNO_DEPS_PRESET` equals the configure preset name unless overridden).

## Prerequisites

| Tool | Notes |
|------|--------|
| CMake ≥ 3.25 | Presets workflow |
| Ninja | Generator in presets |
| GCC / G++ / Gfortran | `linux-gcc-*` and `windows-gcc-*` presets |
| MATLAB + MEX SDK | Required; `find_package(Matlab REQUIRED)` |
| Git | FetchContent for Uno tag when `UNO_MEX_UNO_SOURCE_DIR` unset |

**Windows (MinGW):** MSYS2 `mingw64` bin directory on `PATH` when configuring from PowerShell.

## Presets

| Preset | Use |
|--------|-----|
| `linux-gcc-release-config` | Linux CI / release |
| `linux-gcc-debug-config` | Linux debug (sibling installs must match build type) |
| `windows-gcc-release-config` | Windows release + local paths in preset |
| `windows-gcc-debug-config` | Windows debug; may set `UNO_DEPS_PRESET=windows-gcc-release-config` to reuse release-built siblings |

Shared in-tree options (`uno-mex-in-tree` base): `UNO_MEX_USE_INSTALLED_UNO=OFF`, `UNO_MEX_BUILD_UNO_TESTS=ON`.

## Build steps (local or CI)

Use one preset name everywhere, e.g. `PRESET=linux-gcc-release-config`.

### 1. Build and install numeric dependencies

From any directory, with siblings checked out under `${UNO_DEPS_ROOT}`:

```bash
export UNO_DEPS_ROOT=/path/to/parent   # e.g. /home/runner/work/org/repo or C:/repos/Uno
chmod +x uno_mex/ci/build-superbuild-deps.sh
UNO_DEPS_ROOT="${UNO_DEPS_ROOT}" uno_mex/ci/build-superbuild-deps.sh "${PRESET}"
```

This configures, builds, and installs (in order): **GKlib → METIS → HiGHS → bqpd_lib → coinhsl → MUMPS_cmake → OpenBLAS → MexUtilities**.

Manual equivalent for one library:

```bash
cmake --preset "${PRESET}" -S "${UNO_DEPS_ROOT}/HiGHS"
cmake --build "${UNO_DEPS_ROOT}/HiGHS/out/build/${PRESET}"
cmake --install "${UNO_DEPS_ROOT}/HiGHS/out/build/${PRESET}"
```

**Build order matters:** GKlib before METIS; install all numeric libs before MexUtilities/uno_mex.

### 2. Configure uno_mex

```bash
cd "${UNO_DEPS_ROOT}/uno_mex"

# Linux CI / generic: point at sibling root and Uno checkout
cmake --preset "${PRESET}" \
  -DUNO_DEPS_ROOT="${UNO_DEPS_ROOT}" \
  -DUNO_MEX_UNO_SOURCE_DIR="${UNO_DEPS_ROOT}/Uno" \
  -DUNO_MEXUTILITIES_INSTALL="${UNO_DEPS_ROOT}/MexUtilities/out/install/${PRESET}"

# Windows (paths also in CMakePresets.json for local dev):
# cmake --preset windows-gcc-release-config
```

If `UNO_MEX_UNO_SOURCE_DIR` is omitted, CMake **FetchContent** clones `cvanaret/Uno` at tag `v2.7.2` on first configure.

### 3. Build and test

```bash
cmake --build "out/build/${PRESET}"
ctest --test-dir "out/build/${PRESET}" --output-on-failure
```

Outputs (under `out/build/${PRESET}/`):

| Target | Artifact |
|--------|----------|
| `uno_static` | `_deps/uno_build/libuno.a` (Unix) |
| `uno_mex` | `uno_mex.mexw64` / `uno_mex.mexa64` |
| `uno_options` | `uno_options.mex*` |
| `hs015` | `hs015` / `hs015.exe` |
| `run_unotest` | `_deps/uno_build/run_unotest` (if `UNO_MEX_BUILD_UNO_TESTS=ON`) |

## CI checklist

GitHub Actions reference: [`.github/workflows/build.yml`](.github/workflows/build.yml).

1. **Checkout** `uno_mex`, `Uno` (tag `v2.7.2`), and sibling repos into the same workspace root (`UNO_DEPS_ROOT`).
2. **Toolchain:** `ninja-build`, `g++`, `gfortran` (Linux) or MinGW on `PATH` (Windows).
3. **MATLAB** on the agent; ensure `FindMatlab` can see the install (default system paths or `MATLAB_ROOT`).
4. **Cache** `*/out/install/${PRESET}` for siblings (see workflow `actions/cache` paths).
5. **Build deps** (cache miss): `ci/build-superbuild-deps.sh "${PRESET}"`.
6. **Configure uno_mex** with `UNO_DEPS_ROOT`, `UNO_MEX_UNO_SOURCE_DIR`, `UNO_MEXUTILITIES_INSTALL` as above.
7. **Build** + **ctest**.

### Important cache variables

| Variable | Purpose |
|----------|---------|
| `UNO_DEPS_ROOT` | Parent directory containing sibling repos |
| `UNO_DEPS_PRESET` | Override install subdir name (debug using release deps) |
| `UNO_MEX_UNO_SOURCE_DIR` | Local Uno tree; skips FetchContent |
| `UNO_MEXUTILITIES_INSTALL` | MexUtilities install prefix |
| `UNO_OPENBLAS_INSTALL` | Optional OpenBLAS outside `UNO_DEPS_ROOT` (Windows preset) |
| `UNO_MEX_USE_INSTALLED_UNO` | `ON` = legacy `find_package(Uno)` instead of in-tree build |
| `UNO_MEX_BUILD_UNO_TESTS` | `ON` = build `run_unotest` + GTest |

## Debug vs release siblings

`CMAKE_BUILD_TYPE` for uno_mex must match the **installed** sibling libraries. For a debug uno_mex build against release-installed deps, set:

```bash
-DUNO_DEPS_PRESET=windows-gcc-release-config   # example
```

while using preset `windows-gcc-debug-config`.

## Troubleshooting

| Symptom | Check |
|---------|--------|
| `Could not find Uno C++ sources` | `UNO_MEX_UNO_SOURCE_DIR` or FetchContent network/tag |
| `Could not find MexUtilities` | Build/install MexUtilities; set `UNO_MEXUTILITIES_INSTALL` |
| `Could not find highs` / `mumps` / … | Run `build-superbuild-deps.sh`; same `PRESET` name; `UNO_DEPS_ROOT` |
| `gcc` not found (Windows) | Add `C:\msys64\mingw64\bin` to `PATH` before `cmake` |
| Matlab not found | MATLAB installed; MEX SDK; configure before MinGW `.a`-only suffix hack |
