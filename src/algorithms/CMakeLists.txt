# Define header files
set(HEADER_FILES
    SensorDegradationFilter.hpp
    CollectivePerception.hpp
    BayesCPF.hpp
    EKF.hpp
)

# Define source files
set(SOURCE_FILES
    CollectivePerception.cpp
    BayesCPF.cpp
    EKF.cpp
)

# Add controller library
add_library(${PROJECT_NAME}_algorithms
    SHARED
    ${HEADER_FILES}
    ${SOURCE_FILES}
)

# Copy the header files
install(
    FILES ${HEADER_FILES}
    DESTINATION include/sensor_degradation_filter_real_kheperaiv_nestlab/algorithms
)

# Install the library
install(
    TARGETS ${PROJECT_NAME}_algorithms
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib/sensor_degradation_filter_real_kheperaiv_nestlab
    ARCHIVE DESTINATION lib/sensor_degradation_filter_real_kheperaiv_nestlab
)