protobuf_catkin
===============

A catkin wrapper for Google protocol buffers

## Usage
In your `CMakeLists.txt`, call the protobuf compiler:

```cmake
set(PROTO_DEFNS proto/path-to-my-protos/my-proto.proto)
set(BASE_PATH "proto")
PROTOBUF_CATKIN_GENERATE_CPP2(${BASE_PATH} PROTO_SRCS PROTO_HDRS ${PROTO_DEFNS}) 

...

cs_add_library(${PROJECT_NAME} src/my-source-files.cc
                               ${PROTO_SRCS}
                               ${PROTO_HDRS})
```
where `BASE_PATH` is the base path to the folder containing all protobuf definitions (in this example, the base path is `proto`).

This will generate C++ codes for the protos given by `PROTO_DEFNS`.
The C++ files will be generated under `catkin_wsbuild/${PROJECT_NAME}/compiled_proto`.
The structure within this folder mirrors the structure of the source definitions (within `BASE_PATH`).

From the compiled C++ files, the generated headers (`*.pb.h`) are installed into `catkin_ws/devel/include`, again keeping the folder structure from within `BASE_PATH`.
The `*.proto` definition files are installed into `catkin_ws/devel/share/proto` so that they can be used by other packages.

The example code above would generate the following files:
- `catkin_ws/build/${PROJECT_NAME}/compiled_proto/path-to-my-protos/my-proto.pb.cc`
- `catkin_ws/build/${PROJECT_NAME}/compiled_proto/path-to-my-protos/my-proto.pb.h`
- `catkin_ws/devel/include/path-to-my-protos/my-proto.pb.h`
- `catkin_ws/devel/share/proto/path-to-my-protos/my-proto.pb.h`

The protobuf definitions can then be used with `#include <path-to-my-protos/my-proto.pb.h` in C++ or with `import "path-to-my-protos/my-proto.proto";` in other protobuf definition files.
