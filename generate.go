// Package metrics contains the generated protobuf for the metrics endpoint.
package metrics

// We generate two different files from the API doc, one each for the client and server.

//go:generate protoc --experimental_allow_proto3_optional --go_out=metrics resim/utils/proto/uuid.proto
//go:generate protoc --experimental_allow_proto3_optional --go_out=metrics resim/metrics/proto/metrics.proto
