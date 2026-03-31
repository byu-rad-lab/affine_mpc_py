FetchContent_Declare(ampc_extern
  # TODO: change git tag once there is a release
  GIT_REPOSITORY https://github.com/byu-rad-lab/affine_mpc
  GIT_TAG        origin/main
)

set(AFFINE_MPC_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(AFFINE_MPC_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(AFFINE_MPC_BINDINGS ON CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(ampc_extern)
