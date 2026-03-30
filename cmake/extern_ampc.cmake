FetchContent_Declare(ampc_extern
  # TODO: change git info when it becomes public
  GIT_REPOSITORY github:byu-rad-lab/affine_mpc
  GIT_TAG        origin/main
)

set(affine_mpc_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(affine_mpc_BUILD_EXAMPLE OFF CACHE BOOL "" FORCE)
set(affine_mpc_BINDINGS ON CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(ampc_extern)
