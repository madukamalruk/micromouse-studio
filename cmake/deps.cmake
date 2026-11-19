# Third-party dependencies, pinned by version AND archive hash.
# Upgrade deliberately: change URL + URL_HASH together and note it in docs/decisions.md.
include(FetchContent)
set(FETCHCONTENT_QUIET ON)

# doctest 2.5.3 (MIT) — unit test framework
FetchContent_Declare(doctest
    URL      https://github.com/doctest/doctest/archive/refs/tags/v2.5.3.tar.gz
    URL_HASH SHA256=174ebc4e769928959614789c5b4e9c3d0a0f81a62bb608756b127bfebfb21331
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE)

# nlohmann/json 3.12.0 (MIT) — robot config / reports (used from Slice 3)
FetchContent_Declare(nlohmann_json
    URL      https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz
    URL_HASH SHA256=42f6e95cad6ec532fd372391373363b62a14af6d771056dbfc86160e6dfff7aa
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE)

FetchContent_MakeAvailable(doctest nlohmann_json)

# --- GUI dependencies (Slice 2) ---------------------------------------------
# Dear ImGui 1.93.0, docking branch (MIT)
FetchContent_Declare(imgui
    URL      https://github.com/ocornut/imgui/archive/refs/tags/v1.93.0-docking.tar.gz
    URL_HASH SHA256=746c265540bd3aa109570f29492ef4d8f5b421dd9fd0049c35ca35f21496f1a5
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
# ImPlot 1.1 (MIT)
FetchContent_Declare(implot
    URL      https://github.com/epezent/implot/archive/refs/tags/v1.1.tar.gz
    URL_HASH SHA256=f13e1e111db2df1687e1568644429c2184eb5115867dc76bb0a8bd17a2e72ae8
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
# stb (public domain / MIT) — stb_image_write for PNG screenshots
FetchContent_Declare(stb
    URL      https://github.com/nothings/stb/archive/6b5799bae8a1fe3792691ef4b72ae26434c3e799.tar.gz
    URL_HASH SHA256=87e9ecd2d2e0cb8fa35bf5a93ea6ca3999595ea4f0a4738ed1aac0a34cbd4da9
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
FetchContent_MakeAvailable(imgui implot stb)

find_package(SDL2 REQUIRED CONFIG)

# ImGui + SDL2/SDL_Renderer backends + ImPlot as one static lib (third-party: no -Werror).
add_library(imgui_bundle STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/imgui_demo.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_sdl2.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_sdlrenderer2.cpp
    ${implot_SOURCE_DIR}/implot.cpp
    ${implot_SOURCE_DIR}/implot_items.cpp)
target_include_directories(imgui_bundle SYSTEM PUBLIC
    ${imgui_SOURCE_DIR} ${imgui_SOURCE_DIR}/backends ${implot_SOURCE_DIR})
target_link_libraries(imgui_bundle PUBLIC SDL2::SDL2)

add_library(stb_headers INTERFACE)
target_include_directories(stb_headers SYSTEM INTERFACE ${stb_SOURCE_DIR})
