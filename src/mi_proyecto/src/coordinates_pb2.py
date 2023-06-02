# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: coordinates.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='coordinates.proto',
  package='object_detection',
  syntax='proto3',
  serialized_pb=_b('\n\x0b\x63x100.proto\x12\x10object_detection\"\x07\n\x05\x45mpty\"<\n\x11ObjectCoordinates\x12\t\n\x01x\x18\x01 \x01(\x02\x12\t\n\x01y\x18\x02 \x01(\x02\x12\x11\n\ttimestamp\x18\x03 \x01(\x02\x32n\n\x12GreenObjectService\x12X\n\x18PublishObjectCoordinates\x12\x17.object_detection.Empty\x1a#.object_detection.ObjectCoordinatesb\x06proto3')
)




_EMPTY = _descriptor.Descriptor(
  name='Empty',
  full_name='object_detection.Empty',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=33,
  serialized_end=40,
)


_OBJECTCOORDINATES = _descriptor.Descriptor(
  name='ObjectCoordinates',
  full_name='object_detection.ObjectCoordinates',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='x', full_name='object_detection.ObjectCoordinates.x', index=0,
      number=1, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='y', full_name='object_detection.ObjectCoordinates.y', index=1,
      number=2, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='timestamp', full_name='object_detection.ObjectCoordinates.timestamp', index=2,
      number=3, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=42,
  serialized_end=102,
)

DESCRIPTOR.message_types_by_name['Empty'] = _EMPTY
DESCRIPTOR.message_types_by_name['ObjectCoordinates'] = _OBJECTCOORDINATES
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

Empty = _reflection.GeneratedProtocolMessageType('Empty', (_message.Message,), dict(
  DESCRIPTOR = _EMPTY,
  __module__ = 'coordinates_pb2'
  # @@protoc_insertion_point(class_scope:object_detection.Empty)
  ))
_sym_db.RegisterMessage(Empty)

ObjectCoordinates = _reflection.GeneratedProtocolMessageType('ObjectCoordinates', (_message.Message,), dict(
  DESCRIPTOR = _OBJECTCOORDINATES,
  __module__ = 'coordinates_pb2'
  # @@protoc_insertion_point(class_scope:object_detection.ObjectCoordinates)
  ))
_sym_db.RegisterMessage(ObjectCoordinates)



_GREENOBJECTSERVICE = _descriptor.ServiceDescriptor(
  name='GreenObjectService',
  full_name='object_detection.GreenObjectService',
  file=DESCRIPTOR,
  index=0,
  options=None,
  serialized_start=104,
  serialized_end=214,
  methods=[
  _descriptor.MethodDescriptor(
    name='PublishObjectCoordinates',
    full_name='object_detection.GreenObjectService.PublishObjectCoordinates',
    index=0,
    containing_service=None,
    input_type=_EMPTY,
    output_type=_OBJECTCOORDINATES,
    options=None,
  ),
])
_sym_db.RegisterServiceDescriptor(_GREENOBJECTSERVICE)

DESCRIPTOR.services_by_name['GreenObjectService'] = _GREENOBJECTSERVICE

# @@protoc_insertion_point(module_scope)
