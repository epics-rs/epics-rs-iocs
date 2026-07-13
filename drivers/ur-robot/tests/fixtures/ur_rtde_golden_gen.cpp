// Golden-byte generator: every byte below is produced by ur_rtde's OWN packers
// (RTDEUtility, ur_rtde @ 68ac4e18, include/ur_rtde/rtde_utility.h), compiled
// as-is. The composition of a command frame (header + recipe id + fields) is
// transcribed from RTDE::send (src/rtde.cpp:166-296), which cannot be compiled
// here because it needs boost::asio.
//
// Build (rtde_export.h is CMake-generated; a stub with an empty RTDE_EXPORT is
// enough):
//   printf '#pragma once\n#define RTDE_EXPORT\n' > inc/ur_rtde/rtde_export.h
//   g++ -std=c++17 -Iinc -I<ur_rtde>/include -o gen ur_rtde_golden_gen.cpp
//   ./gen > ur_rtde_golden.rs
#include <ur_rtde/rtde_utility.h>
#include <cstdio>
#include <string>
#include <vector>
using namespace ur_rtde;

static void emit(const char* name, const std::vector<char>& v) {
  printf("pub const %s: &[u8] = &[", name);
  for (size_t i = 0; i < v.size(); ++i)
    printf("%s0x%02x", i ? ", " : "", (unsigned)(unsigned char)v[i]);
  printf("];\n");
}

static void append(std::vector<char>& a, const std::vector<char>& b) {
  a.insert(a.end(), b.begin(), b.end());
}

// RTDE::send: [size u16 BE][cmd u8][recipe u8][payload], size includes header.
static std::vector<char> frame(char cmd, const std::vector<char>& payload) {
  std::vector<char> out;
  uint16_t size = (uint16_t)(3 + payload.size());
  out.push_back((char)(size >> 8));
  out.push_back((char)(size & 0xff));
  out.push_back(cmd);
  append(out, payload);
  return out;
}

int main() {
  printf("// GENERATED from ur_rtde 68ac4e18 include/ur_rtde/rtde_utility.h by\n");
  printf("// compiling its packers and printing their output. Do not edit by hand.\n\n");

  emit("PACK_INT32_1", RTDEUtility::packInt32(1));
  emit("PACK_INT32_MINUS_2", RTDEUtility::packInt32(-2));
  emit("PACK_INT32_MAX", RTDEUtility::packInt32(2147483647));
  emit("PACK_UINT32_DEADBEEF", RTDEUtility::packUInt32(0xdeadbeefu));
  emit("PACK_DOUBLE_ZERO", RTDEUtility::packDouble(0.0));
  emit("PACK_DOUBLE_HALF", RTDEUtility::packDouble(0.5));
  emit("PACK_DOUBLE_MINUS_PI", RTDEUtility::packDouble(-3.14159265358979323846));
  emit("PACK_DOUBLE_125", RTDEUtility::packDouble(125.0));
  emit("PACK_VECTOR6D", RTDEUtility::packVectorNd({0.0, -0.5, 1.5, 2.25, -1e-3, 3.0}));
  emit("PACK_VECTOR3INT32", RTDEUtility::packVectorNInt32({1, -1, 7}));

  // --- whole frames -------------------------------------------------------

  // REQUEST_PROTOCOL_VERSION (86): a u16 BE version.
  {
    std::vector<char> payload;
    payload.push_back((char)(2 >> 8));
    payload.push_back((char)(2 & 0xff));
    emit("FRAME_PROTOCOL_VERSION_2", frame('V', payload));
  }
  // GET_UR_CONTROL_VERSION (118): no payload.
  emit("FRAME_GET_CONTROLLER_VERSION", frame('v', {}));
  // CONTROL_PACKAGE_START (83) / PAUSE (80): no payload.
  emit("FRAME_START", frame('S', {}));
  emit("FRAME_PAUSE", frame('P', {}));
  // CONTROL_PACKAGE_SETUP_OUTPUTS (79): double frequency, then the
  // comma-terminated variable names.
  {
    std::vector<char> payload = RTDEUtility::packDouble(125.0);
    std::string names = "timestamp,actual_q,";
    payload.insert(payload.end(), names.begin(), names.end());
    emit("FRAME_OUTPUT_SETUP_125HZ", frame('O', payload));
  }
  // CONTROL_PACKAGE_SETUP_INPUTS (73): names only.
  {
    std::vector<char> payload;
    std::string names = "input_int_register_23,";
    payload.insert(payload.end(), names.begin(), names.end());
    emit("FRAME_INPUT_SETUP", frame('I', payload));
  }

  // RTDE_DATA_PACKAGE (85), RobotCommand::send order from rtde.cpp:
  //   int32 type, [type-specific fields], val_ vector, async flag; the recipe
  //   id byte is prepended last.
  // MOVEJ (1) on recipe 1: 6 joint doubles + speed + acceleration, then the
  // asynchronous flag as an int32.
  {
    std::vector<char> payload;
    payload.push_back((char)1);                          // recipe id
    append(payload, RTDEUtility::packInt32(1));          // MOVEJ
    append(payload, RTDEUtility::packVectorNd(
        {0.0, -1.5707963267948966, 0.0, -1.5707963267948966, 0.0, 0.0, 1.05, 1.4}));
    append(payload, RTDEUtility::packInt32(1));          // asynchronous
    emit("FRAME_MOVEJ_ASYNC", frame('U', payload));
  }
  // SET_STD_DIGITAL_OUT (13) on recipe 2: mask byte, value byte.
  {
    std::vector<char> payload;
    payload.push_back((char)2);
    append(payload, RTDEUtility::packInt32(13));
    payload.push_back((char)(1u << 3));                  // output 3
    payload.push_back((char)(1u << 3));                  // driven high
    emit("FRAME_SET_STD_DIGITAL_OUT_3_HIGH", frame('U', payload));
  }
  // SET_INPUT_INT_REGISTER (49) on recipe 7 (input_int_register_18): one int32.
  {
    std::vector<char> payload;
    payload.push_back((char)7);
    append(payload, RTDEUtility::packInt32(49));
    append(payload, RTDEUtility::packInt32(-5));
    emit("FRAME_SET_INPUT_INT_REGISTER_18", frame('U', payload));
  }
  // SET_INPUT_DOUBLE_REGISTER (50) on recipe 12 (input_double_register_18).
  {
    std::vector<char> payload;
    payload.push_back((char)12);
    append(payload, RTDEUtility::packInt32(50));
    append(payload, RTDEUtility::packDouble(0.5));
    emit("FRAME_SET_INPUT_DOUBLE_REGISTER_18", frame('U', payload));
  }
  // NO_CMD (0) on recipe 4: nothing but the type.
  {
    std::vector<char> payload;
    payload.push_back((char)4);
    append(payload, RTDEUtility::packInt32(0));
    emit("FRAME_NO_CMD", frame('U', payload));
  }
  // SET_SPEED_SLIDER (22) on the IO interface's recipe 4: mask int32 then the
  // fraction as a double.
  {
    std::vector<char> payload;
    payload.push_back((char)4);
    append(payload, RTDEUtility::packInt32(22));
    append(payload, RTDEUtility::packInt32(1));         // speed_slider_mask
    append(payload, RTDEUtility::packDouble(0.25));
    emit("FRAME_SET_SPEED_SLIDER_25PCT", frame('U', payload));
  }
  return 0;
}
