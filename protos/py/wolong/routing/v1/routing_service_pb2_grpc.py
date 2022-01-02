# Generated by the gRPC Python protocol compiler plugin. DO NOT EDIT!
"""Client and server classes corresponding to protobuf-defined services."""
import grpc

from wolong.routing.v1 import routing_service_pb2 as wolong_dot_routing_dot_v1_dot_routing__service__pb2


class RoutingServiceStub(object):
    """Missing associated documentation comment in .proto file."""

    def __init__(self, channel):
        """Constructor.

        Args:
            channel: A grpc.Channel.
        """
        self.GetRoute = channel.unary_unary(
                '/wolong.routing.v1.RoutingService/GetRoute',
                request_serializer=wolong_dot_routing_dot_v1_dot_routing__service__pb2.GetRouteRequest.SerializeToString,
                response_deserializer=wolong_dot_routing_dot_v1_dot_routing__service__pb2.GetRouteResponse.FromString,
                )


class RoutingServiceServicer(object):
    """Missing associated documentation comment in .proto file."""

    def GetRoute(self, request, context):
        """Get route by type/start/end
        """
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')


def add_RoutingServiceServicer_to_server(servicer, server):
    rpc_method_handlers = {
            'GetRoute': grpc.unary_unary_rpc_method_handler(
                    servicer.GetRoute,
                    request_deserializer=wolong_dot_routing_dot_v1_dot_routing__service__pb2.GetRouteRequest.FromString,
                    response_serializer=wolong_dot_routing_dot_v1_dot_routing__service__pb2.GetRouteResponse.SerializeToString,
            ),
    }
    generic_handler = grpc.method_handlers_generic_handler(
            'wolong.routing.v1.RoutingService', rpc_method_handlers)
    server.add_generic_rpc_handlers((generic_handler,))


 # This class is part of an EXPERIMENTAL API.
class RoutingService(object):
    """Missing associated documentation comment in .proto file."""

    @staticmethod
    def GetRoute(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/wolong.routing.v1.RoutingService/GetRoute',
            wolong_dot_routing_dot_v1_dot_routing__service__pb2.GetRouteRequest.SerializeToString,
            wolong_dot_routing_dot_v1_dot_routing__service__pb2.GetRouteResponse.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)
