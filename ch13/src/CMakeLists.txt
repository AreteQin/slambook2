add_library(myslam SHARED
        frame.cpp
        mappoint.cpp
        map.cpp
        camera.cpp
        config.cpp
        feature.cpp
        frontend.cpp
        backend.cpp
        viewer.cpp
        visual_odometry.cpp)
target_link_libraries(myslam ${THIRD_PARTY_LIBS})

add_library(eth3d_dataset eth3d_dataset.cpp)
target_link_libraries(eth3d_dataset myslam)

add_library(kitti_dataset kitti_dataset.cpp)
target_link_libraries(kitti_dataset myslam)
