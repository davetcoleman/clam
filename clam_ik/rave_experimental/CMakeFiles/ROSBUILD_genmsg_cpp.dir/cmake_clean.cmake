FILE(REMOVE_RECURSE
  "msg_gen"
  "src/rave_experimental/msg"
  "msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "msg_gen/cpp/include/rave_experimental/CameraPosition.h"
  "msg_gen/cpp/include/rave_experimental/GrabResponse.h"
  "msg_gen/cpp/include/rave_experimental/GrabCommand.h"
  "msg_gen/cpp/include/rave_experimental/BodyStatusRequest.h"
  "msg_gen/cpp/include/rave_experimental/CameraPositionRequest.h"
  "msg_gen/cpp/include/rave_experimental/JointRequest.h"
  "msg_gen/cpp/include/rave_experimental/PutCommand.h"
  "msg_gen/cpp/include/rave_experimental/EnviromentUpdate.h"
  "msg_gen/cpp/include/rave_experimental/HandUpdate.h"
  "msg_gen/cpp/include/rave_experimental/HandRequest.h"
  "msg_gen/cpp/include/rave_experimental/JointState.h"
  "msg_gen/cpp/include/rave_experimental/PutResponse.h"
  "msg_gen/cpp/include/rave_experimental/JointUpdate.h"
  "msg_gen/cpp/include/rave_experimental/BodyStatus.h"
  "msg_gen/cpp/include/rave_experimental/control.h"
  "msg_gen/cpp/include/rave_experimental/HandState.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
