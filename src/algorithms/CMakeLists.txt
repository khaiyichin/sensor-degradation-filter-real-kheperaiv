# Add collective perception library
add_library(${PROJECT_NAME}_algorithms
    SHARED
    SensorDegradationFilter.hpp
    CollectivePerception.hpp
    CollectivePerception.cpp
    BayesCPF.hpp
    BayesCPF.cpp
    EKF.hpp
    EKF.cpp
)