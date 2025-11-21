add_executable(YggDemo main.cpp)
file(COPY shaders DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

target_link_libraries(YggDemo
    PRIVATE Ygg
)
