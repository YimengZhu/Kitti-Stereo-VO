add_library( myslam SHARED
    ORBextractor.cpp
    frame.cpp
    mappoint.cpp
    map.cpp
    camera.cpp
    config.cpp
    g2o_types.cpp
    vo.cpp
)

target_link_libraries( myslam
    ${THIRD_PARTY_LIBS}
)
