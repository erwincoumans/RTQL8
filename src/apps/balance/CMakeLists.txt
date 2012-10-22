###############################################
# apps/balance
PROJECT(balance)
FILE(GLOB collision_srcs "*.cpp")
FILE(GLOB collision_hdrs "*.h")
ADD_EXECUTABLE(balance ${collision_srcs} ${collision_hdrs})
TARGET_LINK_LIBRARIES(balance lcpsolver collision kinematics dynamics yui renderer integration)
LINK_THIRDPARTY_GLUT(balance)
LINK_THIRDPARTY_TICPP(balance)

SET_TARGET_PROPERTIES(balance PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
