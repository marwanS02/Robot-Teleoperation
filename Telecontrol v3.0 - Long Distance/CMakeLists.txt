# 1. Minimum required CMake version
cmake_minimum_required(VERSION 3.20)

# 2. Project name and language
project(udp_tx_rx_sines_imgui CXX)

# 3. C++ standard settings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 4. Add include directories (optional)
include(FetchContent)

# ImGui
FetchContent_Declare(
  imgui
  GIT_REPOSITORY https://github.com/ocornut/imgui.git
  GIT_TAG        v1.91.5
)
FetchContent_MakeAvailable(imgui)

# ImPlot
FetchContent_Declare(
  implot
  GIT_REPOSITORY https://github.com/epezent/implot.git
  GIT_TAG        v0.16
)
FetchContent_MakeAvailable(implot)

# Build ImGui + backends (Win32 + DX11)
add_library(imgui_win32_dx11
  ${imgui_SOURCE_DIR}/imgui.cpp
  ${imgui_SOURCE_DIR}/imgui_draw.cpp
  ${imgui_SOURCE_DIR}/imgui_widgets.cpp
  ${imgui_SOURCE_DIR}/imgui_tables.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_win32.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_dx11.cpp
)
target_include_directories(imgui_win32_dx11 PUBLIC
  ${imgui_SOURCE_DIR}
  ${imgui_SOURCE_DIR}/backends
)

# ImPlot (needs to see imgui headers)
add_library(implot_lib
  ${implot_SOURCE_DIR}/implot.cpp
  ${implot_SOURCE_DIR}/implot_items.cpp
)
target_include_directories(implot_lib PUBLIC
  ${implot_SOURCE_DIR}
  ${imgui_SOURCE_DIR}   # <--- critical line
)
target_link_libraries(implot_lib PUBLIC imgui_win32_dx11)


# 5. Define the executable (or library)
add_executable(udp_tx_rx_sines_imgui main.cpp)

# 6. Link external libraries (optional)
target_link_libraries(udp_tx_rx_sines_imgui PRIVATE imgui_win32_dx11 implot_lib)

if (WIN32)
  target_compile_definitions(udp_tx_rx_sines_imgui PRIVATE
    _CRT_SECURE_NO_WARNINGS
    NOMINMAX
    IMGUI_IMPL_WIN32_DISABLE_GAMEPAD
    IMGUI_DISABLE_OBSOLETE_FUNCTIONS
  )
  target_link_libraries(udp_tx_rx_sines_imgui PRIVATE
    ws2_32
    d3d11
    d3dcompiler
    dxgi
  )
endif()
