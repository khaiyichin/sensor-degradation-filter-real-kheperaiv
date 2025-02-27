# Add controller executable
add_executable(bayes_cpf_diffusion_controller BayesCPFDiffusionController.hpp BayesCPFDiffusionController.cpp /opt/poky/1.8/sysroots/cortexa8hf-vfp-neon-poky-linux-gnueabi/usr/include/argos3/plugins/robots/kheperaiv/real_robot/main.cpp)

target_link_libraries(bayes_cpf_diffusion_controller
    argos3core_khiv
    argos3plugin_khiv_kheperaiv
    argos3plugin_khiv_genericrobot
    ${PROJECT_NAME}_algorithms
    ${PROJECT_NAME}_messages
)

# Copy the header files
install(
    FILES BayesCPFDiffusionController.hpp /opt/poky/1.8/sysroots/cortexa8hf-vfp-neon-poky-linux-gnueabi/usr/include/argos3/plugins/robots/kheperaiv/real_robot/main.cpp
    DESTINATION include/sensor_degradation_filter_real_kheperaiv_nestlab/controllers
)

# Install the executable
install(
    TARGETS bayes_cpf_diffusion_controller
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib/sensor_degradation_filter_real_kheperaiv_nestlab
    ARCHIVE DESTINATION lib/sensor_degradation_filter_real_kheperaiv_nestlab
)