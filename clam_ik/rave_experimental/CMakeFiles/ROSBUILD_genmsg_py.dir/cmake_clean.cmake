FILE(REMOVE_RECURSE
  "msg_gen"
  "src/rave_experimental/msg"
  "msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "src/rave_experimental/msg/__init__.py"
  "src/rave_experimental/msg/_CameraPosition.py"
  "src/rave_experimental/msg/_GrabResponse.py"
  "src/rave_experimental/msg/_GrabCommand.py"
  "src/rave_experimental/msg/_BodyStatusRequest.py"
  "src/rave_experimental/msg/_CameraPositionRequest.py"
  "src/rave_experimental/msg/_JointRequest.py"
  "src/rave_experimental/msg/_PutCommand.py"
  "src/rave_experimental/msg/_EnviromentUpdate.py"
  "src/rave_experimental/msg/_HandUpdate.py"
  "src/rave_experimental/msg/_HandRequest.py"
  "src/rave_experimental/msg/_JointState.py"
  "src/rave_experimental/msg/_PutResponse.py"
  "src/rave_experimental/msg/_JointUpdate.py"
  "src/rave_experimental/msg/_BodyStatus.py"
  "src/rave_experimental/msg/_control.py"
  "src/rave_experimental/msg/_HandState.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
