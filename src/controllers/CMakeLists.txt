
add_executable(bayes_cpf_diffusion BayesCPFDiffusionController.hpp BayesCPFDiffusionController.cpp /opt/poky/1.8/sysroots/cortexa8hf-vfp-neon-poky-linux-gnueabi/usr/include/argos3/plugins/robots/kheperaiv/real_robot/main.cpp)

target_link_libraries(bayes_cpf_diffusion
    argos3core_khiv
    argos3plugin_khiv_kheperaiv
    argos3plugin_khiv_genericrobot
    ${PROJECT_NAME}_algorithms
)