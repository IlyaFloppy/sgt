protoc -I . --grpc_out=. --plugin=protoc-gen-grpc=$(which grpc_cpp_plugin) ./*.proto
protoc -I . --cpp_out=. ./*.proto

# binaries at https://github.com/grpc/grpc-swift/releases
protoc --swift_out=. *.proto
# protoc --swiftgrpc_out=. *.proto # swift grpc cgrpc (old) branch
#protoc --swiftgrpc_out=. --plugin=protoc-gen-grpc="$(./protoc-gen-grpc-swift)" *.proto # swift grpc main (new) branch

# add protoc-gen-grpc-swift to path
export PATH=$(pwd):$PATH
# swift grpc main (new) branch
protoc video.proto --proto_path=. --plugin=protoc-gen-grpc=./protoc-gen-grpc-swift --grpc-swift_out=.
