import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from .wolong.routing.v1 import routing_pb2
from .wolong.routing.v1 import routing_service_pb2
from .wolong.routing.v1 import routing_service_pb2_grpc
