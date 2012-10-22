message (STATUS "Setting up dependecies configurations")

# third party libraries
if (UNIX)
    message(STATUS "Operating system = Unix")
    INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/thirdparty/eigen-3.0.5)
    INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/thirdparty/ticpp)

    LINK_DIRECTORIES(${CMAKE_SOURCE_DIR}/thirdparty/lib)
    LINK_DIRECTORIES(${CMAKE_SOURCE_DIR}/bin)
else()
    message(STATUS "Operating system = WIN32")
    INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/thirdparty/eigen-3.0.5)
    INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/thirdparty/glut-3.7.6/include)
    INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/thirdparty/ticpp)

    LINK_DIRECTORIES(${CMAKE_SOURCE_DIR}/thirdparty/lib)
endif()

# glut
if (WIN32)
    INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/thirdparty/glut-3.7.6/include)
endif()
