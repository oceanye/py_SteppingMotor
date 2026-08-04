#pragma once

// Small, Arduino-independent strict numeric parsers.  Keeping these helpers
// free of String/Serial dependencies makes the protocol boundary testable in
// PlatformIO's native environment.
#include <cerrno>
#include <climits>
#include <cmath>
#include <cstdlib>
#include <cctype>

inline bool protocol_token_has_space(const char* text) {
  if (text == nullptr) return true;
  for (const unsigned char* p = reinterpret_cast<const unsigned char*>(text); *p; ++p) {
    if (std::isspace(*p)) return true;
  }
  return false;
}

inline int protocol_count_fields(const char* line) {
  if (line == nullptr || *line == '\0') return 0;
  int fields = 1;
  for (const char* p = line; *p; ++p) {
    if (*p == ',') ++fields;
  }
  return fields;
}

inline bool protocol_parse_long(const char* text, long min_value,
                                long max_value, long& result) {
  if (text == nullptr || *text == '\0' || min_value > max_value ||
      protocol_token_has_space(text)) return false;
  errno = 0;
  char* end = nullptr;
  const long value = std::strtol(text, &end, 10);
  if (errno == ERANGE || end == text || *end != '\0' ||
      value < min_value || value > max_value) return false;
  result = value;
  return true;
}

inline bool protocol_parse_int(const char* text, int min_value,
                               int max_value, int& result) {
  long value = 0;
  if (!protocol_parse_long(text, min_value, max_value, value)) return false;
  result = static_cast<int>(value);
  return true;
}

inline bool protocol_parse_float(const char* text, float min_value,
                                 float max_value, float& result) {
  if (text == nullptr || *text == '\0' || !(min_value <= max_value) ||
      protocol_token_has_space(text)) return false;
  errno = 0;
  char* end = nullptr;
  const float value = std::strtof(text, &end);
  if (errno == ERANGE || end == text || *end != '\0' || !std::isfinite(value) ||
      value < min_value || value > max_value) return false;
  result = value;
  return true;
}
