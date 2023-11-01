// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: healthinfo.proto

#include "healthinfo.pb.h"

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
namespace Colossus {
namespace Protobuf {
class HealthInfoDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<HealthInfo> _instance;
} _HealthInfo_default_instance_;
}  // namespace Protobuf
}  // namespace Colossus
static void InitDefaultsscc_info_HealthInfo_healthinfo_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::Colossus::Protobuf::_HealthInfo_default_instance_;
    new (ptr) ::Colossus::Protobuf::HealthInfo();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::Colossus::Protobuf::HealthInfo::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_HealthInfo_healthinfo_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 0, 0, InitDefaultsscc_info_HealthInfo_healthinfo_2eproto}, {}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_healthinfo_2eproto[1];
static const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* file_level_enum_descriptors_healthinfo_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_healthinfo_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_healthinfo_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::Colossus::Protobuf::HealthInfo, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::Colossus::Protobuf::HealthInfo, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::Colossus::Protobuf::HealthInfo, minvalue_),
  PROTOBUF_FIELD_OFFSET(::Colossus::Protobuf::HealthInfo, maxvalue_),
  PROTOBUF_FIELD_OFFSET(::Colossus::Protobuf::HealthInfo, value_),
  PROTOBUF_FIELD_OFFSET(::Colossus::Protobuf::HealthInfo, delta_),
  PROTOBUF_FIELD_OFFSET(::Colossus::Protobuf::HealthInfo, status_),
  PROTOBUF_FIELD_OFFSET(::Colossus::Protobuf::HealthInfo, warnallowance_),
  PROTOBUF_FIELD_OFFSET(::Colossus::Protobuf::HealthInfo, alarmallowance_),
  PROTOBUF_FIELD_OFFSET(::Colossus::Protobuf::HealthInfo, clearallowance_),
  2,
  3,
  0,
  1,
  4,
  5,
  6,
  7,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 13, sizeof(::Colossus::Protobuf::HealthInfo)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::Colossus::Protobuf::_HealthInfo_default_instance_),
};

const char descriptor_table_protodef_healthinfo_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\020healthinfo.proto\022\021Colossus.Protobuf\"\351\001"
  "\n\nHealthInfo\022\025\n\010minvalue\030\001 \001(\002:\003-10\022\024\n\010m"
  "axvalue\030\002 \001(\002:\00210\022\020\n\005value\030\003 \001(\002:\0010\022\020\n\005d"
  "elta\030\004 \001(\002:\0010\0228\n\006status\030\005 \001(\0162\037.Colossus"
  ".Protobuf.HealthStatus:\007UNKNOWN\022\031\n\rwarna"
  "llowance\030\006 \001(\005:\00212\022\032\n\016alarmallowance\030\007 \001"
  "(\005:\00220\022\031\n\016clearallowance\030\010 \001(\005:\0011*D\n\014Hea"
  "lthStatus\022\r\n\tUNHEALTHY\020\000\022\013\n\007WARNING\020\001\022\013\n"
  "\007HEALTHY\020\002\022\013\n\007UNKNOWN\020\003"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_healthinfo_2eproto_deps[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_healthinfo_2eproto_sccs[1] = {
  &scc_info_HealthInfo_healthinfo_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_healthinfo_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_healthinfo_2eproto = {
  false, false, descriptor_table_protodef_healthinfo_2eproto, "healthinfo.proto", 343,
  &descriptor_table_healthinfo_2eproto_once, descriptor_table_healthinfo_2eproto_sccs, descriptor_table_healthinfo_2eproto_deps, 1, 0,
  schemas, file_default_instances, TableStruct_healthinfo_2eproto::offsets,
  file_level_metadata_healthinfo_2eproto, 1, file_level_enum_descriptors_healthinfo_2eproto, file_level_service_descriptors_healthinfo_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_healthinfo_2eproto = (static_cast<void>(::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_healthinfo_2eproto)), true);
namespace Colossus {
namespace Protobuf {
const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* HealthStatus_descriptor() {
  ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&descriptor_table_healthinfo_2eproto);
  return file_level_enum_descriptors_healthinfo_2eproto[0];
}
bool HealthStatus_IsValid(int value) {
  switch (value) {
    case 0:
    case 1:
    case 2:
    case 3:
      return true;
    default:
      return false;
  }
}


// ===================================================================

void HealthInfo::InitAsDefaultInstance() {
}
class HealthInfo::_Internal {
 public:
  using HasBits = decltype(std::declval<HealthInfo>()._has_bits_);
  static void set_has_minvalue(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
  static void set_has_maxvalue(HasBits* has_bits) {
    (*has_bits)[0] |= 8u;
  }
  static void set_has_value(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static void set_has_delta(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static void set_has_status(HasBits* has_bits) {
    (*has_bits)[0] |= 16u;
  }
  static void set_has_warnallowance(HasBits* has_bits) {
    (*has_bits)[0] |= 32u;
  }
  static void set_has_alarmallowance(HasBits* has_bits) {
    (*has_bits)[0] |= 64u;
  }
  static void set_has_clearallowance(HasBits* has_bits) {
    (*has_bits)[0] |= 128u;
  }
};

HealthInfo::HealthInfo(::PROTOBUF_NAMESPACE_ID::Arena* arena)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena) {
  SharedCtor();
  RegisterArenaDtor(arena);
  // @@protoc_insertion_point(arena_constructor:Colossus.Protobuf.HealthInfo)
}
HealthInfo::HealthInfo(const HealthInfo& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  ::memcpy(&value_, &from.value_,
    static_cast<size_t>(reinterpret_cast<char*>(&clearallowance_) -
    reinterpret_cast<char*>(&value_)) + sizeof(clearallowance_));
  // @@protoc_insertion_point(copy_constructor:Colossus.Protobuf.HealthInfo)
}

void HealthInfo::SharedCtor() {
  ::memset(&value_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&delta_) -
      reinterpret_cast<char*>(&value_)) + sizeof(delta_));
  minvalue_ = -10;
  maxvalue_ = 10;
  status_ = 3;
  warnallowance_ = 12;
  alarmallowance_ = 20;
  clearallowance_ = 1;
}

HealthInfo::~HealthInfo() {
  // @@protoc_insertion_point(destructor:Colossus.Protobuf.HealthInfo)
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

void HealthInfo::SharedDtor() {
  GOOGLE_DCHECK(GetArena() == nullptr);
}

void HealthInfo::ArenaDtor(void* object) {
  HealthInfo* _this = reinterpret_cast< HealthInfo* >(object);
  (void)_this;
}
void HealthInfo::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void HealthInfo::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const HealthInfo& HealthInfo::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_HealthInfo_healthinfo_2eproto.base);
  return *internal_default_instance();
}


void HealthInfo::Clear() {
// @@protoc_insertion_point(message_clear_start:Colossus.Protobuf.HealthInfo)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x000000ffu) {
    ::memset(&value_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&delta_) -
        reinterpret_cast<char*>(&value_)) + sizeof(delta_));
    minvalue_ = -10;
    maxvalue_ = 10;
    status_ = 3;
    warnallowance_ = 12;
    alarmallowance_ = 20;
    clearallowance_ = 1;
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* HealthInfo::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  ::PROTOBUF_NAMESPACE_ID::Arena* arena = GetArena(); (void)arena;
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // optional float minvalue = 1 [default = -10];
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 13)) {
          _Internal::set_has_minvalue(&has_bits);
          minvalue_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<float>(ptr);
          ptr += sizeof(float);
        } else goto handle_unusual;
        continue;
      // optional float maxvalue = 2 [default = 10];
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 21)) {
          _Internal::set_has_maxvalue(&has_bits);
          maxvalue_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<float>(ptr);
          ptr += sizeof(float);
        } else goto handle_unusual;
        continue;
      // optional float value = 3 [default = 0];
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 29)) {
          _Internal::set_has_value(&has_bits);
          value_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<float>(ptr);
          ptr += sizeof(float);
        } else goto handle_unusual;
        continue;
      // optional float delta = 4 [default = 0];
      case 4:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 37)) {
          _Internal::set_has_delta(&has_bits);
          delta_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<float>(ptr);
          ptr += sizeof(float);
        } else goto handle_unusual;
        continue;
      // optional .Colossus.Protobuf.HealthStatus status = 5 [default = UNKNOWN];
      case 5:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 40)) {
          ::PROTOBUF_NAMESPACE_ID::uint64 val = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
          if (PROTOBUF_PREDICT_TRUE(::Colossus::Protobuf::HealthStatus_IsValid(val))) {
            _internal_set_status(static_cast<::Colossus::Protobuf::HealthStatus>(val));
          } else {
            ::PROTOBUF_NAMESPACE_ID::internal::WriteVarint(5, val, mutable_unknown_fields());
          }
        } else goto handle_unusual;
        continue;
      // optional int32 warnallowance = 6 [default = 12];
      case 6:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 48)) {
          _Internal::set_has_warnallowance(&has_bits);
          warnallowance_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional int32 alarmallowance = 7 [default = 20];
      case 7:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 56)) {
          _Internal::set_has_alarmallowance(&has_bits);
          alarmallowance_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional int32 clearallowance = 8 [default = 1];
      case 8:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 64)) {
          _Internal::set_has_clearallowance(&has_bits);
          clearallowance_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      default: {
      handle_unusual:
        if ((tag & 7) == 4 || tag == 0) {
          ctx->SetLastTag(tag);
          goto success;
        }
        ptr = UnknownFieldParse(tag,
            _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(),
            ptr, ctx);
        CHK_(ptr != nullptr);
        continue;
      }
    }  // switch
  }  // while
success:
  _has_bits_.Or(has_bits);
  return ptr;
failure:
  ptr = nullptr;
  goto success;
#undef CHK_
}

::PROTOBUF_NAMESPACE_ID::uint8* HealthInfo::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:Colossus.Protobuf.HealthInfo)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional float minvalue = 1 [default = -10];
  if (cached_has_bits & 0x00000004u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteFloatToArray(1, this->_internal_minvalue(), target);
  }

  // optional float maxvalue = 2 [default = 10];
  if (cached_has_bits & 0x00000008u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteFloatToArray(2, this->_internal_maxvalue(), target);
  }

  // optional float value = 3 [default = 0];
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteFloatToArray(3, this->_internal_value(), target);
  }

  // optional float delta = 4 [default = 0];
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteFloatToArray(4, this->_internal_delta(), target);
  }

  // optional .Colossus.Protobuf.HealthStatus status = 5 [default = UNKNOWN];
  if (cached_has_bits & 0x00000010u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteEnumToArray(
      5, this->_internal_status(), target);
  }

  // optional int32 warnallowance = 6 [default = 12];
  if (cached_has_bits & 0x00000020u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteInt32ToArray(6, this->_internal_warnallowance(), target);
  }

  // optional int32 alarmallowance = 7 [default = 20];
  if (cached_has_bits & 0x00000040u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteInt32ToArray(7, this->_internal_alarmallowance(), target);
  }

  // optional int32 clearallowance = 8 [default = 1];
  if (cached_has_bits & 0x00000080u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteInt32ToArray(8, this->_internal_clearallowance(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:Colossus.Protobuf.HealthInfo)
  return target;
}

size_t HealthInfo::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:Colossus.Protobuf.HealthInfo)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x000000ffu) {
    // optional float value = 3 [default = 0];
    if (cached_has_bits & 0x00000001u) {
      total_size += 1 + 4;
    }

    // optional float delta = 4 [default = 0];
    if (cached_has_bits & 0x00000002u) {
      total_size += 1 + 4;
    }

    // optional float minvalue = 1 [default = -10];
    if (cached_has_bits & 0x00000004u) {
      total_size += 1 + 4;
    }

    // optional float maxvalue = 2 [default = 10];
    if (cached_has_bits & 0x00000008u) {
      total_size += 1 + 4;
    }

    // optional .Colossus.Protobuf.HealthStatus status = 5 [default = UNKNOWN];
    if (cached_has_bits & 0x00000010u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::EnumSize(this->_internal_status());
    }

    // optional int32 warnallowance = 6 [default = 12];
    if (cached_has_bits & 0x00000020u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::Int32Size(
          this->_internal_warnallowance());
    }

    // optional int32 alarmallowance = 7 [default = 20];
    if (cached_has_bits & 0x00000040u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::Int32Size(
          this->_internal_alarmallowance());
    }

    // optional int32 clearallowance = 8 [default = 1];
    if (cached_has_bits & 0x00000080u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::Int32Size(
          this->_internal_clearallowance());
    }

  }
  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    return ::PROTOBUF_NAMESPACE_ID::internal::ComputeUnknownFieldsSize(
        _internal_metadata_, total_size, &_cached_size_);
  }
  int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void HealthInfo::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:Colossus.Protobuf.HealthInfo)
  GOOGLE_DCHECK_NE(&from, this);
  const HealthInfo* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<HealthInfo>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:Colossus.Protobuf.HealthInfo)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:Colossus.Protobuf.HealthInfo)
    MergeFrom(*source);
  }
}

void HealthInfo::MergeFrom(const HealthInfo& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:Colossus.Protobuf.HealthInfo)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x000000ffu) {
    if (cached_has_bits & 0x00000001u) {
      value_ = from.value_;
    }
    if (cached_has_bits & 0x00000002u) {
      delta_ = from.delta_;
    }
    if (cached_has_bits & 0x00000004u) {
      minvalue_ = from.minvalue_;
    }
    if (cached_has_bits & 0x00000008u) {
      maxvalue_ = from.maxvalue_;
    }
    if (cached_has_bits & 0x00000010u) {
      status_ = from.status_;
    }
    if (cached_has_bits & 0x00000020u) {
      warnallowance_ = from.warnallowance_;
    }
    if (cached_has_bits & 0x00000040u) {
      alarmallowance_ = from.alarmallowance_;
    }
    if (cached_has_bits & 0x00000080u) {
      clearallowance_ = from.clearallowance_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void HealthInfo::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:Colossus.Protobuf.HealthInfo)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void HealthInfo::CopyFrom(const HealthInfo& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:Colossus.Protobuf.HealthInfo)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool HealthInfo::IsInitialized() const {
  return true;
}

void HealthInfo::InternalSwap(HealthInfo* other) {
  using std::swap;
  _internal_metadata_.Swap<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(HealthInfo, delta_)
      + sizeof(HealthInfo::delta_)
      - PROTOBUF_FIELD_OFFSET(HealthInfo, value_)>(
          reinterpret_cast<char*>(&value_),
          reinterpret_cast<char*>(&other->value_));
  swap(minvalue_, other->minvalue_);
  swap(maxvalue_, other->maxvalue_);
  swap(status_, other->status_);
  swap(warnallowance_, other->warnallowance_);
  swap(alarmallowance_, other->alarmallowance_);
  swap(clearallowance_, other->clearallowance_);
}

::PROTOBUF_NAMESPACE_ID::Metadata HealthInfo::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace Protobuf
}  // namespace Colossus
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::Colossus::Protobuf::HealthInfo* Arena::CreateMaybeMessage< ::Colossus::Protobuf::HealthInfo >(Arena* arena) {
  return Arena::CreateMessageInternal< ::Colossus::Protobuf::HealthInfo >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
