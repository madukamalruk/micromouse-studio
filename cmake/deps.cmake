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

# ImGui (docking) + ImPlot are added in Slice 2.
