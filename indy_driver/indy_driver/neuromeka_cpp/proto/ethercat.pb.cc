// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ethercat.proto

#include "ethercat.pb.h"

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
namespace Nrmk {
namespace IndyFramework {
}  // namespace IndyFramework
}  // namespace Nrmk
static constexpr const ::_pb::EnumDescriptor**
    file_level_enum_descriptors_ethercat_2eproto = nullptr;
static constexpr const ::_pb::ServiceDescriptor**
    file_level_service_descriptors_ethercat_2eproto = nullptr;
const ::uint32_t TableStruct_ethercat_2eproto::offsets[1] = {};
static constexpr ::_pbi::MigrationSchema* schemas = nullptr;
static constexpr ::_pb::Message* const* file_default_instances = nullptr;
const char descriptor_table_protodef_ethercat_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
    "\n\016ethercat.proto\022\022Nrmk.IndyFramework\032\023et"
    "hercat_msgs.proto\032\021common_msgs.proto2\203\036\n"
    "\010EtherCAT\022P\n\017GetMasterStatus\022\031.Nrmk.Indy"
    "Framework.Empty\032 .Nrmk.IndyFramework.Mas"
    "terStatus\"\000\022N\n\016GetSlaveStatus\022\031.Nrmk.Ind"
    "yFramework.Empty\032\037.Nrmk.IndyFramework.Sl"
    "aveStatus\"\000\022V\n\021GetRxDomainStatus\022\031.Nrmk."
    "IndyFramework.Empty\032$.Nrmk.IndyFramework"
    ".EcatDomainStatus\"\000\022V\n\021GetTxDomainStatus"
    "\022\031.Nrmk.IndyFramework.Empty\032$.Nrmk.IndyF"
    "ramework.EcatDomainStatus\"\000\022Q\n\rIsSystemR"
    "eady\022\031.Nrmk.IndyFramework.Empty\032#.Nrmk.I"
    "ndyFramework.EcatSystemReady\"\000\022I\n\tIsServ"
    "oOn\022\031.Nrmk.IndyFramework.Empty\032\037.Nrmk.In"
    "dyFramework.EcatServoOn\"\000\022P\n\017GetSlaveTyp"
    "eNum\022\031.Nrmk.IndyFramework.Empty\032 .Nrmk.I"
    "ndyFramework.SlaveTypeNum\"\000\022L\n\022ResetOver"
    "flowCount\022\031.Nrmk.IndyFramework.Empty\032\031.N"
    "rmk.IndyFramework.Empty\"\000\022K\n\nSetServoRx\022"
    " .Nrmk.IndyFramework.ServoRxIndex\032\031.Nrmk"
    ".IndyFramework.Empty\"\000\022K\n\nGetServoRx\022\036.N"
    "rmk.IndyFramework.ServoIndex\032\033.Nrmk.Indy"
    "Framework.ServoRx\"\000\022K\n\nGetServoTx\022\036.Nrmk"
    ".IndyFramework.ServoIndex\032\033.Nrmk.IndyFra"
    "mework.ServoTx\"\000\022S\n\016SetServoRxKeba\022$.Nrm"
    "k.IndyFramework.ServoRxIndexKeba\032\031.Nrmk."
    "IndyFramework.Empty\"\000\022S\n\016GetServoRxKeba\022"
    "\036.Nrmk.IndyFramework.ServoIndex\032\037.Nrmk.I"
    "ndyFramework.ServoRxKeba\"\000\022S\n\016GetServoTx"
    "Keba\022\036.Nrmk.IndyFramework.ServoIndex\032\037.N"
    "rmk.IndyFramework.ServoTxKeba\"\000\022I\n\nSetSe"
    "rvoOn\022\036.Nrmk.IndyFramework.ServoIndex\032\031."
    "Nrmk.IndyFramework.Empty\"\000\022J\n\013SetServoOf"
    "f\022\036.Nrmk.IndyFramework.ServoIndex\032\031.Nrmk"
    ".IndyFramework.Empty\"\000\022V\n\023GetServoTemper"
    "ature\022\036.Nrmk.IndyFramework.ServoIndex\032\035."
    "Nrmk.IndyFramework.ServoTemp\"\000\022U\n\021GetSer"
    "voErrorCode\022\036.Nrmk.IndyFramework.ServoIn"
    "dex\032\036.Nrmk.IndyFramework.ServoError\"\000\022I\n"
    "\nResetServo\022\036.Nrmk.IndyFramework.ServoIn"
    "dex\032\031.Nrmk.IndyFramework.Empty\"\000\022Q\n\022SetC"
    "OREManualBrake\022\036.Nrmk.IndyFramework.Serv"
    "oBrake\032\031.Nrmk.IndyFramework.Empty\"\000\022J\n\014S"
    "etEndtoolRx\022\035.Nrmk.IndyFramework.Endtool"
    "Rx\032\031.Nrmk.IndyFramework.Empty\"\000\022J\n\014GetEn"
    "dtoolRx\022\031.Nrmk.IndyFramework.Empty\032\035.Nrm"
    "k.IndyFramework.EndtoolRx\"\000\022J\n\014GetEndtoo"
    "lTx\022\031.Nrmk.IndyFramework.Empty\032\035.Nrmk.In"
    "dyFramework.EndtoolTx\"\000\022X\n\023GetEndtoolDoc"
    "kingTx\022\031.Nrmk.IndyFramework.Empty\032$.Nrmk"
    ".IndyFramework.EndtoolDockingTx\"\000\022T\n\021Set"
    "EndtoolRS485Rx\022\".Nrmk.IndyFramework.Endt"
    "oolRS485Rx\032\031.Nrmk.IndyFramework.Empty\"\000\022"
    "T\n\021GetEndtoolRS485Rx\022\031.Nrmk.IndyFramewor"
    "k.Empty\032\".Nrmk.IndyFramework.EndtoolRS48"
    "5Rx\"\000\022T\n\021GetEndtoolRS485Tx\022\031.Nrmk.IndyFr"
    "amework.Empty\032\".Nrmk.IndyFramework.Endto"
    "olRS485Tx\"\000\022K\n\020SetEndtoolLedDim\022\032.Nrmk.I"
    "ndyFramework.LedDim\032\031.Nrmk.IndyFramework"
    ".Empty\"\000\022T\n\021SetSRKeyEndtoolRx\022\".Nrmk.Ind"
    "yFramework.SRKeyEndtoolRx\032\031.Nrmk.IndyFra"
    "mework.Empty\"\000\022T\n\021GetSRKeyEndtoolRx\022\031.Nr"
    "mk.IndyFramework.Empty\032\".Nrmk.IndyFramew"
    "ork.SRKeyEndtoolRx\"\000\022T\n\021GetSRKeyEndtoolT"
    "x\022\031.Nrmk.IndyFramework.Empty\032\".Nrmk.Indy"
    "Framework.SRKeyEndtoolTx\"\000\022J\n\014SetIOBoard"
    "Rx\022\035.Nrmk.IndyFramework.IOBoardRx\032\031.Nrmk"
    ".IndyFramework.Empty\"\000\022J\n\014GetIOBoardTx\022\031"
    ".Nrmk.IndyFramework.Empty\032\035.Nrmk.IndyFra"
    "mework.IOBoardTx\"\000\022J\n\014GetIOBoardRx\022\031.Nrm"
    "k.IndyFramework.Empty\032\035.Nrmk.IndyFramewo"
    "rk.IOBoardRx\"\000\022L\n\005GetDI\022\034.Nrmk.IndyFrame"
    "work.DIOIndex\032#.Nrmk.IndyFramework.DIODi"
    "gitalInput\"\000\022M\n\005GetDO\022\034.Nrmk.IndyFramewo"
    "rk.DIOIndex\032$.Nrmk.IndyFramework.DIODigi"
    "talOutput\"\000\022J\n\005SetDO\022$.Nrmk.IndyFramewor"
    "k.DIODigitalOutput\032\031.Nrmk.IndyFramework."
    "Empty\"\000\022Q\n\017GetMaxTorqueSDO\022\035.Nrmk.IndyFr"
    "amework.EcatIndex\032\035.Nrmk.IndyFramework.S"
    "DOIntVal\"\000\022R\n\020GetProfileVelSDO\022\035.Nrmk.In"
    "dyFramework.EcatIndex\032\035.Nrmk.IndyFramewo"
    "rk.SDOIntVal\"\000\022R\n\020GetProfileAccSDO\022\035.Nrm"
    "k.IndyFramework.EcatIndex\032\035.Nrmk.IndyFra"
    "mework.SDOIntVal\"\000\022R\n\020GetProfileDecSDO\022\035"
    ".Nrmk.IndyFramework.EcatIndex\032\035.Nrmk.Ind"
    "yFramework.SDOIntVal\"\000\022N\n\017SetMaxTorqueSD"
    "O\022\036.Nrmk.IndyFramework.ServoParam\032\031.Nrmk"
    ".IndyFramework.Empty\"\000\022O\n\020SetProfileVelS"
    "DO\022\036.Nrmk.IndyFramework.ServoParam\032\031.Nrm"
    "k.IndyFramework.Empty\"\000\022O\n\020SetProfileAcc"
    "SDO\022\036.Nrmk.IndyFramework.ServoParam\032\031.Nr"
    "mk.IndyFramework.Empty\"\000\022O\n\020SetProfileDe"
    "cSDO\022\036.Nrmk.IndyFramework.ServoParam\032\031.N"
    "rmk.IndyFramework.Empty\"\000\022Y\n\021GetRobotZer"
    "oCount\022\036.Nrmk.IndyFramework.ServoIndex\032\""
    ".Nrmk.IndyFramework.RobotZeroCount\"\000\022T\n\025"
    "SetRobotZeroAsCurrent\022\036.Nrmk.IndyFramewo"
    "rk.ServoIndex\032\031.Nrmk.IndyFramework.Empty"
    "\"\000b\006proto3"
};
static const ::_pbi::DescriptorTable* const descriptor_table_ethercat_2eproto_deps[2] =
    {
        &::descriptor_table_common_5fmsgs_2eproto,
        &::descriptor_table_ethercat_5fmsgs_2eproto,
};
static ::absl::once_flag descriptor_table_ethercat_2eproto_once;
const ::_pbi::DescriptorTable descriptor_table_ethercat_2eproto = {
    false,
    false,
    3930,
    descriptor_table_protodef_ethercat_2eproto,
    "ethercat.proto",
    &descriptor_table_ethercat_2eproto_once,
    descriptor_table_ethercat_2eproto_deps,
    2,
    0,
    schemas,
    file_default_instances,
    TableStruct_ethercat_2eproto::offsets,
    nullptr,
    file_level_enum_descriptors_ethercat_2eproto,
    file_level_service_descriptors_ethercat_2eproto,
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
PROTOBUF_ATTRIBUTE_WEAK const ::_pbi::DescriptorTable* descriptor_table_ethercat_2eproto_getter() {
  return &descriptor_table_ethercat_2eproto;
}
// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY2
static ::_pbi::AddDescriptorsRunner dynamic_init_dummy_ethercat_2eproto(&descriptor_table_ethercat_2eproto);
namespace Nrmk {
namespace IndyFramework {
// @@protoc_insertion_point(namespace_scope)
}  // namespace IndyFramework
}  // namespace Nrmk
namespace google {
namespace protobuf {
}  // namespace protobuf
}  // namespace google
// @@protoc_insertion_point(global_scope)
#include "google/protobuf/port_undef.inc"
