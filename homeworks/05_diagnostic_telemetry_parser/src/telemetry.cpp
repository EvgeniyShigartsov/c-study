#include "../include/telemetry.hpp"

#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>

// Debugging exercise notes:
// this file intentionally contains four runtime defects.
// The defects are related to malformed input shape, invalid numeric values,
// unsafe time deltas, and empty logs. Exact locations are not marked on purpose.

const int EXPECTED_FIELD_COUNT = 7;
const int MAX_LINE_LENGTH = 256;

const char* FIELD_NAMES[EXPECTED_FIELD_COUNT] = {
  "timestamp_ms",
  "seq",
  "voltage_v",
  "current_a",
  "temperature_c",
  "gps_fix",
  "satellites",
};

int split_line(char line[], char* fields[], int max_fields)
{
  int count = 0;
  char* cursor = line;

  while (*cursor != '\0' && count < max_fields) {
    while (*cursor == ' ' || *cursor == '\t' || *cursor == '\n' || *cursor == '\r') {
      *cursor = '\0';
      ++cursor;
    }

    if (*cursor == '\0') {
      break;
    }

    fields[count] = cursor;
    ++count;

    while (*cursor != '\0' && *cursor != ' ' && *cursor != '\t' && *cursor != '\n' && *cursor != '\r') {
      ++cursor;
    }
  }

  return count;
}

bool parse_long(const char* text, long& out_value)
{
  char* end = nullptr;
  out_value = std::strtol(text, &end, 10);

  return end == text ? false : true;
}

bool parse_int(const char* text, int& out_value)
{
  long longN;

  const bool isValid = parse_long(text, longN);
  out_value = static_cast<int>(longN);

  return isValid;
}

bool parse_double(const char* text, double& out_value)
{
  char* end = nullptr;
  out_value = std::strtod(text, &end);

  return end == text ? false : true;
}

bool parse_frame(char line[], const int frameN, Frame& out_frame)
{
  char* fields[EXPECTED_FIELD_COUNT] = {};
  const int field_count = split_line(line, fields, EXPECTED_FIELD_COUNT);
  (void)field_count;

  if (field_count != EXPECTED_FIELD_COUNT) {
    std::cerr << "error: invalid frame at line " << frameN << ", expected " << EXPECTED_FIELD_COUNT << " fields but found: " << field_count
              << std::endl;
    return false;
  }

  bool errorsFound = false;
  char errorMsg[300];
  sprintf(errorMsg, "Invalid frame values at line [%d] - ", frameN);

  Frame localFrame = {};

  const bool parseResults[EXPECTED_FIELD_COUNT] = {
    parse_long(fields[0], localFrame.timestamp_ms),
    parse_int(fields[1], localFrame.seq),
    parse_double(fields[2], localFrame.voltage_v),
    parse_double(fields[3], localFrame.current_a),
    parse_double(fields[4], localFrame.temperature_c),
    parse_int(fields[5], localFrame.gps_fix),
    parse_int(fields[6], localFrame.satellites),
  };

  for (int i = 0; i < EXPECTED_FIELD_COUNT; i++) {
    if (!parseResults[i]) {
      strcat(errorMsg, FIELD_NAMES[i]);
      strcat(errorMsg, ", ");

      errorsFound = true;
    }
  }

  if (errorsFound) {
    std::cerr << errorMsg << std::endl;
    return false;
  }

  out_frame = localFrame;

  return true;
}

double compute_frame_rate_hz(const Frame frames[], int frame_count)
{
  const long elapsed_ms = frames[frame_count - 1].timestamp_ms - frames[0].timestamp_ms;

  return static_cast<double>((frame_count - 1) * 1000 / elapsed_ms);
}

int read_frames(const char* path, Frame frames[], int max_frames)
{
  std::ifstream input{path};
  if (!input) {
    std::cerr << "error: failed to open input file: " << path << '\n';
    return 0;
  }

  int frame_count = 0;
  char line[MAX_LINE_LENGTH];

  bool isAllFramesValid = true;

  while (input.getline(line, MAX_LINE_LENGTH)) {
    if (frame_count < max_frames) {
      const bool isFrameValid = parse_frame(line, frame_count + 1, frames[frame_count]);
      ++frame_count;

      if (!isFrameValid) {
        isAllFramesValid = false;
      }
    }
  }

  if (!isAllFramesValid) {
    return 0;
  }

  return frame_count;
}

bool validateParsedFrames(const Frame frames[], int framesCout)
{
  bool errorsFound = false;

  for (int i = 0; i < framesCout; i++) {
    const Frame currentFrame = frames[i];
    const Frame prevFrame = frames[i - 1];
    bool isFrameValid = true;

    bool frameValidationResults[EXPECTED_FIELD_COUNT] = {
      i == 0 ? true : currentFrame.timestamp_ms > prevFrame.timestamp_ms,
      i == 0 ? true : currentFrame.seq - prevFrame.seq == 1,
      currentFrame.voltage_v > 0,
      true,  // current_a does not require validation
      currentFrame.temperature_c > -40 || currentFrame.temperature_c < 120,
      currentFrame.gps_fix == 1 || currentFrame.gps_fix == 0,
      currentFrame.satellites >= 0,
    };

    char frameErrorMsg[400];
    sprintf(frameErrorMsg, "Frame validation failed at line [%d], check validation rules. Invalid values: ", i + 1);

    for (int j = 0; j < EXPECTED_FIELD_COUNT; j++) {
      if (!frameValidationResults[j]) {
        strcat(frameErrorMsg, FIELD_NAMES[j]);
        strcat(frameErrorMsg, ", ");

        isFrameValid = false;
      }
    }

    if (!isFrameValid) {
      std::cerr << frameErrorMsg << std::endl;
      errorsFound = true;
    }
  }
  return !errorsFound;
}

Summary summarize(const Frame frames[], int frame_count)
{
  Summary summary{};
  summary.frames_total = frame_count;
  summary.frames_valid = frame_count;
  summary.voltage_min = frames[0].voltage_v;
  summary.voltage_max = frames[0].voltage_v;
  summary.low_voltage_frames = 0;

  double temperature_sum = 0.0;

  for (int i = 0; i < frame_count; ++i) {
    if (frames[i].voltage_v < summary.voltage_min) {
      summary.voltage_min = frames[i].voltage_v;
    }

    if (frames[i].voltage_v > summary.voltage_max) {
      summary.voltage_max = frames[i].voltage_v;
    }

    temperature_sum += frames[i].temperature_c;

    if (frames[i].voltage_v < 22.0) {
      ++summary.low_voltage_frames;
    }
  }

  const int temperature_tenths = static_cast<int>(temperature_sum * 10.0) / frame_count;
  summary.temperature_avg = static_cast<double>(temperature_tenths) / 10.0;
  summary.frame_rate_hz = compute_frame_rate_hz(frames, frame_count);
  return summary;
}

void print_summary(const Summary& summary)
{
  std::cout << "frames_total " << summary.frames_total << '\n';
  std::cout << "frames_valid " << summary.frames_valid << '\n';
  std::cout << "voltage_min " << summary.voltage_min << '\n';
  std::cout << "voltage_max " << summary.voltage_max << '\n';
  std::cout << "temperature_avg " << summary.temperature_avg << '\n';
  std::cout << "low_voltage_frames " << summary.low_voltage_frames << '\n';
  std::cout << "frame_rate_hz " << summary.frame_rate_hz << '\n';
}
