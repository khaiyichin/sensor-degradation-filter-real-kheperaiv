add_library(${PROJECT_NAME}_messages
    INTERFACE
)

target_include_directories(${PROJECT_NAME}_messages
    INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/messages>
)