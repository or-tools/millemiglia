// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: int128.proto

#include "int128.pb.h"

#include <algorithm>
#include "google/protobuf/io/coded_stream.h"
#include "google/protobuf/extension_set.h"
#include "google/protobuf/wire_format_lite.h"
#include "google/protobuf/descriptor.h"
#include "google/protobuf/generated_message_reflection.h"
#include "google/protobuf/reflection_ops.h"
#include "google/protobuf/wire_format.h"
#include "google/protobuf/generated_message_tctable_impl.h"
// @@protoc_insertion_point(includes)

// Must be included last.
#include "google/protobuf/port_def.inc"
PROTOBUF_PRAGMA_INIT_SEG
namespace _pb = ::google::protobuf;
namespace _pbi = ::google::protobuf::internal;
namespace _fl = ::google::protobuf::internal::field_layout;
namespace storage_util {

inline constexpr Int128::Impl_::Impl_(
    ::_pbi::ConstantInitialized) noexcept
      : high_{::int64_t{0}},
        low_{::uint64_t{0u}},
        _cached_size_{0} {}

template <typename>
PROTOBUF_CONSTEXPR Int128::Int128(::_pbi::ConstantInitialized)
    : _impl_(::_pbi::ConstantInitialized()) {}
struct Int128DefaultTypeInternal {
  PROTOBUF_CONSTEXPR Int128DefaultTypeInternal() : _instance(::_pbi::ConstantInitialized{}) {}
  ~Int128DefaultTypeInternal() {}
  union {
    Int128 _instance;
  };
};

PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT
    PROTOBUF_ATTRIBUTE_INIT_PRIORITY1 Int128DefaultTypeInternal _Int128_default_instance_;
}  // namespace storage_util
static ::_pb::Metadata file_level_metadata_int128_2eproto[1];
static constexpr const ::_pb::EnumDescriptor**
    file_level_enum_descriptors_int128_2eproto = nullptr;
static constexpr const ::_pb::ServiceDescriptor**
    file_level_service_descriptors_int128_2eproto = nullptr;
const ::uint32_t TableStruct_int128_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(
    protodesc_cold) = {
    ~0u,  // no _has_bits_
    PROTOBUF_FIELD_OFFSET(::storage_util::Int128, _internal_metadata_),
    ~0u,  // no _extensions_
    ~0u,  // no _oneof_case_
    ~0u,  // no _weak_field_map_
    ~0u,  // no _inlined_string_donated_
    ~0u,  // no _split_
    ~0u,  // no sizeof(Split)
    PROTOBUF_FIELD_OFFSET(::storage_util::Int128, _impl_.high_),
    PROTOBUF_FIELD_OFFSET(::storage_util::Int128, _impl_.low_),
};

static const ::_pbi::MigrationSchema
    schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
        {0, -1, -1, sizeof(::storage_util::Int128)},
};

static const ::_pb::Message* const file_default_instances[] = {
    &::storage_util::_Int128_default_instance_._instance,
};
const char descriptor_table_protodef_int128_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
    "\n\014int128.proto\022\014storage_util\"+\n\006Int128\022\020"
    "\n\004high\030\001 \001(\003B\0020\001\022\017\n\003low\030\002 \001(\004B\0020\001B\033\n\027com"
    ".google.storage.utilP\001b\006proto3"
};
static ::absl::once_flag descriptor_table_int128_2eproto_once;
const ::_pbi::DescriptorTable descriptor_table_int128_2eproto = {
    false,
    false,
    110,
    descriptor_table_protodef_int128_2eproto,
    "int128.proto",
    &descriptor_table_int128_2eproto_once,
    nullptr,
    0,
    1,
    schemas,
    file_default_instances,
    TableStruct_int128_2eproto::offsets,
    file_level_metadata_int128_2eproto,
    file_level_enum_descriptors_int128_2eproto,
    file_level_service_descriptors_int128_2eproto,
};

// This function exists to be marked as weak.
// It can significantly speed up compilation by breaking up LLVM's SCC
// in the .pb.cc translation units. Large translation units see a
// reduction of more than 35% of walltime for optimized builds. Without
// the weak attribute all the messages in the file, including all the
// vtables and everything they use become part of the same SCC through
// a cycle like:
// GetMetadata -> descriptor table -> default instances ->
//   vtables -> GetMetadata
// By adding a weak function here we break the connection from the
// individual vtables back into the descriptor table.
PROTOBUF_ATTRIBUTE_WEAK const ::_pbi::DescriptorTable* descriptor_table_int128_2eproto_getter() {
  return &descriptor_table_int128_2eproto;
}
// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY2
static ::_pbi::AddDescriptorsRunner dynamic_init_dummy_int128_2eproto(&descriptor_table_int128_2eproto);
namespace storage_util {
// ===================================================================

class Int128::_Internal {
 public:
};

Int128::Int128(::google::protobuf::Arena* arena)
    : ::google::protobuf::Message(arena) {
  SharedCtor(arena);
  // @@protoc_insertion_point(arena_constructor:storage_util.Int128)
}
Int128::Int128(
    ::google::protobuf::Arena* arena, const Int128& from)
    : Int128(arena) {
  MergeFrom(from);
}
inline PROTOBUF_NDEBUG_INLINE Int128::Impl_::Impl_(
    ::google::protobuf::internal::InternalVisibility visibility,
    ::google::protobuf::Arena* arena)
      : _cached_size_{0} {}

inline void Int128::SharedCtor(::_pb::Arena* arena) {
  new (&_impl_) Impl_(internal_visibility(), arena);
  ::memset(reinterpret_cast<char *>(&_impl_) +
               offsetof(Impl_, high_),
           0,
           offsetof(Impl_, low_) -
               offsetof(Impl_, high_) +
               sizeof(Impl_::low_));
}
Int128::~Int128() {
  // @@protoc_insertion_point(destructor:storage_util.Int128)
  _internal_metadata_.Delete<::google::protobuf::UnknownFieldSet>();
  SharedDtor();
}
inline void Int128::SharedDtor() {
  ABSL_DCHECK(GetArena() == nullptr);
  _impl_.~Impl_();
}

PROTOBUF_NOINLINE void Int128::Clear() {
// @@protoc_insertion_point(message_clear_start:storage_util.Int128)
  PROTOBUF_TSAN_WRITE(&_impl_._tsan_detect_race);
  ::uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  ::memset(&_impl_.high_, 0, static_cast<::size_t>(
      reinterpret_cast<char*>(&_impl_.low_) -
      reinterpret_cast<char*>(&_impl_.high_)) + sizeof(_impl_.low_));
  _internal_metadata_.Clear<::google::protobuf::UnknownFieldSet>();
}

const char* Int128::_InternalParse(
    const char* ptr, ::_pbi::ParseContext* ctx) {
  ptr = ::_pbi::TcParser::ParseLoop(this, ptr, ctx, &_table_.header);
  return ptr;
}


PROTOBUF_CONSTINIT PROTOBUF_ATTRIBUTE_INIT_PRIORITY1
const ::_pbi::TcParseTable<1, 2, 0, 0, 2> Int128::_table_ = {
  {
    0,  // no _has_bits_
    0, // no _extensions_
    2, 8,  // max_field_number, fast_idx_mask
    offsetof(decltype(_table_), field_lookup_table),
    4294967292,  // skipmap
    offsetof(decltype(_table_), field_entries),
    2,  // num_field_entries
    0,  // num_aux_entries
    offsetof(decltype(_table_), field_names),  // no aux_entries
    &_Int128_default_instance_._instance,
    ::_pbi::TcParser::GenericFallback,  // fallback
  }, {{
    // uint64 low = 2 [jstype = JS_STRING];
    {::_pbi::TcParser::SingularVarintNoZag1<::uint64_t, offsetof(Int128, _impl_.low_), 63>(),
     {16, 63, 0, PROTOBUF_FIELD_OFFSET(Int128, _impl_.low_)}},
    // int64 high = 1 [jstype = JS_STRING];
    {::_pbi::TcParser::SingularVarintNoZag1<::uint64_t, offsetof(Int128, _impl_.high_), 63>(),
     {8, 63, 0, PROTOBUF_FIELD_OFFSET(Int128, _impl_.high_)}},
  }}, {{
    65535, 65535
  }}, {{
    // int64 high = 1 [jstype = JS_STRING];
    {PROTOBUF_FIELD_OFFSET(Int128, _impl_.high_), 0, 0,
    (0 | ::_fl::kFcSingular | ::_fl::kInt64)},
    // uint64 low = 2 [jstype = JS_STRING];
    {PROTOBUF_FIELD_OFFSET(Int128, _impl_.low_), 0, 0,
    (0 | ::_fl::kFcSingular | ::_fl::kUInt64)},
  }},
  // no aux_entries
  {{
  }},
};

::uint8_t* Int128::_InternalSerialize(
    ::uint8_t* target,
    ::google::protobuf::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:storage_util.Int128)
  ::uint32_t cached_has_bits = 0;
  (void)cached_has_bits;

  // int64 high = 1 [jstype = JS_STRING];
  if (this->_internal_high() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::
        WriteInt64ToArrayWithField<1>(
            stream, this->_internal_high(), target);
  }

  // uint64 low = 2 [jstype = JS_STRING];
  if (this->_internal_low() != 0) {
    target = stream->EnsureSpace(target);
    target = ::_pbi::WireFormatLite::WriteUInt64ToArray(
        2, this->_internal_low(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target =
        ::_pbi::WireFormat::InternalSerializeUnknownFieldsToArray(
            _internal_metadata_.unknown_fields<::google::protobuf::UnknownFieldSet>(::google::protobuf::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:storage_util.Int128)
  return target;
}

::size_t Int128::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:storage_util.Int128)
  ::size_t total_size = 0;

  ::uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // int64 high = 1 [jstype = JS_STRING];
  if (this->_internal_high() != 0) {
    total_size += ::_pbi::WireFormatLite::Int64SizePlusOne(
        this->_internal_high());
  }

  // uint64 low = 2 [jstype = JS_STRING];
  if (this->_internal_low() != 0) {
    total_size += ::_pbi::WireFormatLite::UInt64SizePlusOne(
        this->_internal_low());
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_impl_._cached_size_);
}

const ::google::protobuf::Message::ClassData Int128::_class_data_ = {
    Int128::MergeImpl,
    nullptr,  // OnDemandRegisterArenaDtor
};
const ::google::protobuf::Message::ClassData* Int128::GetClassData() const {
  return &_class_data_;
}

void Int128::MergeImpl(::google::protobuf::Message& to_msg, const ::google::protobuf::Message& from_msg) {
  auto* const _this = static_cast<Int128*>(&to_msg);
  auto& from = static_cast<const Int128&>(from_msg);
  // @@protoc_insertion_point(class_specific_merge_from_start:storage_util.Int128)
  ABSL_DCHECK_NE(&from, _this);
  ::uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  if (from._internal_high() != 0) {
    _this->_internal_set_high(from._internal_high());
  }
  if (from._internal_low() != 0) {
    _this->_internal_set_low(from._internal_low());
  }
  _this->_internal_metadata_.MergeFrom<::google::protobuf::UnknownFieldSet>(from._internal_metadata_);
}

void Int128::CopyFrom(const Int128& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:storage_util.Int128)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

PROTOBUF_NOINLINE bool Int128::IsInitialized() const {
  return true;
}

::_pbi::CachedSize* Int128::AccessCachedSize() const {
  return &_impl_._cached_size_;
}
void Int128::InternalSwap(Int128* PROTOBUF_RESTRICT other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  ::google::protobuf::internal::memswap<
      PROTOBUF_FIELD_OFFSET(Int128, _impl_.low_)
      + sizeof(Int128::_impl_.low_)
      - PROTOBUF_FIELD_OFFSET(Int128, _impl_.high_)>(
          reinterpret_cast<char*>(&_impl_.high_),
          reinterpret_cast<char*>(&other->_impl_.high_));
}

::google::protobuf::Metadata Int128::GetMetadata() const {
  return ::_pbi::AssignDescriptors(
      &descriptor_table_int128_2eproto_getter, &descriptor_table_int128_2eproto_once,
      file_level_metadata_int128_2eproto[0]);
}
// @@protoc_insertion_point(namespace_scope)
}  // namespace storage_util
namespace google {
namespace protobuf {
}  // namespace protobuf
}  // namespace google
// @@protoc_insertion_point(global_scope)
#include "google/protobuf/port_undef.inc"