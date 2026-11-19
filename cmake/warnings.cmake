# Strict warnings for our own targets only (never applied to third-party code).
function(mmsim_set_warnings target)
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Werror)
endfunction()
