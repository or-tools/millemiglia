// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: latlng.proto

#include "latlng.pb.h"

#include <algorithm>

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/wire_format_lite.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>

PROTOBUF_PRAGMA_INIT_SEG

namespace _pb = ::PROTOBUF_NAMESPACE_ID;
namespace _pbi = _pb::internal;

namespace google {
namespace type {
PROTOBUF_CONSTEXPR LatLng::LatLng(
    ::_pbi::ConstantInitialized): _impl_{
    /*decltype(_impl_.latitude_)*/0
  , /*decltype(_impl_.longitude_)*/0
  , /*decltype(_impl_._cached_size_)*/{}} {}
struct LatLngDefaultTypeInternal {
  PROTOBUF_CONSTEXPR LatLngDefaultTypeInternal()
      : _instance(::_pbi::ConstantInitialized{}) {}
  ~LatLngDefaultTypeInternal() {}
  union {
    LatLng _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT PROTOBUF_ATTRIBUTE_INIT_PRIORITY1 LatLngDefaultTypeInternal _LatLng_default_instance_;
}  // namespace type
}  // namespace google
static ::_pb::Metadata file_level_metadata_latlng_2eproto[1];
static constexpr ::_pb::EnumDescriptor const** file_level_enum_descriptors_latlng_2eproto = nullptr;
static constexpr ::_pb::ServiceDescriptor const** file_level_service_descriptors_latlng_2eproto = nullptr;

const uint32_t TableStruct_latlng_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::google::type::LatLng, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::google::type::LatLng, _impl_.latitude_),
  PROTOBUF_FIELD_OFFSET(::google::type::LatLng, _impl_.longitude_),
};
static const ::_pbi::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, -1, sizeof(::google::type::LatLng)},
};

static const ::_pb::Message* const file_default_instances[] = {
  &::google::type::_LatLng_default_instance_._instance,
};

const char descriptor_table_protodef_latlng_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\014latlng.proto\022\013google.type\"-\n\006LatLng\022\020\n"
  "\010latitude\030\001 \001(\001\022\021\n\tlongitude\030\002 \001(\001Bc\n\017co"
  "m.google.typeB\013LatLngProtoP\001Z8google.gol"
  "ang.org/genproto/googleapis/type/latlng;"
  "latlng\370\001\001\242\002\003GTPb\006proto3"
  ;
static ::_pbi::once_flag descriptor_table_latlng_2eproto_once;
const ::_pbi::DescriptorTable descriptor_table_latlng_2eproto = {
    false, false, 183, descriptor_table_protodef_latlng_2eproto,
    "latlng.proto",
    &descriptor_table_latlng_2eproto_once, nullptr, 0, 1,
    schemas, file_default_instances, TableStruct_latlng_2eproto::offsets,
    file_level_metadata_latlng_2eproto, file_level_enum_descriptors_latlng_2eproto,
    file_level_service_descriptors_latlng_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::_pbi::DescriptorTable* descriptor_table_latlng_2eproto_getter() {
  return &descriptor_table_latlng_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY2 static ::_pbi::AddDescriptorsRunner dynamic_init_dummy_latlng_2eproto(&descriptor_table_latlng_2eproto);
namespace google {
namespace type {

// ===================================================================

class LatLng::_Internal {
 public:
};

LatLng::LatLng(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor(arena, is_message_owned);
  // @@protoc_insertion_point(arena_constructor:google.type.LatLng)
}
LatLng::LatLng(const LatLng& from)
  : ::PROTOBUF_NAMESPACE_ID::Message() {
  LatLng* const _this = this; (void)_this;
  new (&_impl_) Impl_{
      decltype(_impl_.latitude_){}
    , decltype(_impl_.longitude_){}
    , /*decltype(_impl_._cached_size_)*/{}};

  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  ::memcpy(&_impl_.latitude_, &from._impl_.latitude_,
    static_cast<size_t>(reinterpret_cast<char*>(&_impl_.longitude_) -
    reinterpret_cast<char*>(&_impl_.latitude_)) + sizeof(_impl_.longitude_));
  // @@protoc_insertion_point(copy_constructor:google.type.LatLng)
}

inline void LatLng::SharedCtor(
    ::_pb::Arena* arena, bool is_message_owned) {
  (void)arena;
  (void)is_message_owned;
  new (&_impl_) Impl_{
      decltype(_impl_.latitude_){0}
    , decltype(_impl_.longitude_){0}
    , /*decltype(_impl_._cached_size_)*/{}
  };
}

LatLng::~LatLng() {
  // @@protoc_insertion_point(destructor:google.type.LatLng)
  if (auto *arena = _internal_metadata_.DeleteReturnArena<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>()) {
  (void)arena;
    return;
  }
  SharedDtor();
}

inline void LatLng::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
}

void LatLng::SetCachedSize(int size) const {
  _impl_._cached_size_.Set(size);
}

void LatLng::Clear() {
// @@protoc_insertion_point(message_clear_start:google.type.LatLng)
  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  ::memset(&_impl_.latitude_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&_impl_.longitude_) -
      reinterpret_cast<char*>(&_impl_.latitude_)) + sizeof(_impl_.longitude_));
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* LatLng::_InternalParse(const char* ptr, ::_pbi::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    uint32_t tag;
    ptr = ::_pbi::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // double latitude = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 9)) {
          _impl_.latitude_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      // double longitude = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 17)) {
          _impl_.longitude_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      default:
        goto handle_unusual;
    }  // switch
  handle_unusual:
    if ((tag == 0) || ((tag & 7) == 4)) {
      CHK_(ptr);
      ctx->SetLastTag(tag);
      goto message_done;
    }
    ptr = UnknownFieldParse(
        tag,
        _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(),
        ptr, ctx);
    CHK_(ptr != nullptr);
  }  // while
message_done:
  return ptr;
failure:
  ptr = nullptr;
  goto message_done;
#undef CHK_
}

uint8_t* LatLng::_InternalSerialize(
    uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:google.type.LatLng)
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  // double latitude = 1;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_latitude = this->_internal_latitude();
  uint64_t raw_latitude;
  memcpy(&raw_latitude, &tmp_latitude, sizeof(tmp_latitude));
  if (raw_latitude != 0) {
    target = stream->EnsureSpace(target);
    target = ::_pbi::WireFormatLite::WriteDoubleToArray(1, this->_internal_latitude(), target);
  }

  // double longitude = 2;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_longitude = this->_internal_longitude();
  uint64_t raw_longitude;
  memcpy(&raw_longitude, &tmp_longitude, sizeof(tmp_longitude));
  if (raw_longitude != 0) {
    target = stream->EnsureSpace(target);
    target = ::_pbi::WireFormatLite::WriteDoubleToArray(2, this->_internal_longitude(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::_pbi::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:google.type.LatLng)
  return target;
}

size_t LatLng::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:google.type.LatLng)
  size_t total_size = 0;

  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // double latitude = 1;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_latitude = this->_internal_latitude();
  uint64_t raw_latitude;
  memcpy(&raw_latitude, &tmp_latitude, sizeof(tmp_latitude));
  if (raw_latitude != 0) {
    total_size += 1 + 8;
  }

  // double longitude = 2;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_longitude = this->_internal_longitude();
  uint64_t raw_longitude;
  memcpy(&raw_longitude, &tmp_longitude, sizeof(tmp_longitude));
  if (raw_longitude != 0) {
    total_size += 1 + 8;
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_impl_._cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData LatLng::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSourceCheck,
    LatLng::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*LatLng::GetClassData() const { return &_class_data_; }


void LatLng::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message& to_msg, const ::PROTOBUF_NAMESPACE_ID::Message& from_msg) {
  auto* const _this = static_cast<LatLng*>(&to_msg);
  auto& from = static_cast<const LatLng&>(from_msg);
  // @@protoc_insertion_point(class_specific_merge_from_start:google.type.LatLng)
  GOOGLE_DCHECK_NE(&from, _this);
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_latitude = from._internal_latitude();
  uint64_t raw_latitude;
  memcpy(&raw_latitude, &tmp_latitude, sizeof(tmp_latitude));
  if (raw_latitude != 0) {
    _this->_internal_set_latitude(from._internal_latitude());
  }
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_longitude = from._internal_longitude();
  uint64_t raw_longitude;
  memcpy(&raw_longitude, &tmp_longitude, sizeof(tmp_longitude));
  if (raw_longitude != 0) {
    _this->_internal_set_longitude(from._internal_longitude());
  }
  _this->_internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void LatLng::CopyFrom(const LatLng& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:google.type.LatLng)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool LatLng::IsInitialized() const {
  return true;
}

void LatLng::InternalSwap(LatLng* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(LatLng, _impl_.longitude_)
      + sizeof(LatLng::_impl_.longitude_)
      - PROTOBUF_FIELD_OFFSET(LatLng, _impl_.latitude_)>(
          reinterpret_cast<char*>(&_impl_.latitude_),
          reinterpret_cast<char*>(&other->_impl_.latitude_));
}

::PROTOBUF_NAMESPACE_ID::Metadata LatLng::GetMetadata() const {
  return ::_pbi::AssignDescriptors(
      &descriptor_table_latlng_2eproto_getter, &descriptor_table_latlng_2eproto_once,
      file_level_metadata_latlng_2eproto[0]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace type
}  // namespace google
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::google::type::LatLng*
Arena::CreateMaybeMessage< ::google::type::LatLng >(Arena* arena) {
  return Arena::CreateMessageInternal< ::google::type::LatLng >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>