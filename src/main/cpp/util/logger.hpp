#pragma once
#include <chrono>
#include <cstdio>
#include <map>
#include <string>

namespace util {
namespace logging {
enum class LogLevel {
  UNKNOWN = 0,
  TRACE = 5,
  DEBUG = 10,
  INFO = 20,
  WARNING = 30,
  ERROR = 40,
  CRITICAL = 50,
  SUCCESS = 60,
  NONE = 100,
};

extern LogLevel log_level;

std::map<std::string, LogLevel> const log_level_map{
    {"trace", LogLevel::TRACE},     {"debug", LogLevel::DEBUG}, {"info", LogLevel::INFO},
    {"warning", LogLevel::WARNING}, {"error", LogLevel::ERROR}, {"critical", LogLevel::CRITICAL},
    {"success", LogLevel::SUCCESS}, {"none", LogLevel::NONE},
};
}  // namespace logging

void print_trace_prefix();
void print_debug_prefix();
void print_info_prefix();
void print_warning_prefix();
void print_error_prefix();
void print_critical_prefix();
void print_success_prefix();

void print_trace_postfix();
void print_debug_postfix();
void print_info_postfix();
void print_warning_postfix();
void print_error_postfix();
void print_critical_postfix();
void print_success_postfix();

void set_log_level(util::logging::LogLevel log_level);
void set_colored_log(bool enabled);

void timer_start(int key = 0);
double timer_stop(int key = 0);
}  // namespace util

//==================================================================================================
// Logging macros
//==================================================================================================
// clang-format off
#ifdef NDEBUG
#define log_trace(format, ...)
#define log_trace_(format)
#else
#define log_trace(format, ...) {if (util::logging::LogLevel::TRACE >= util::logging::log_level) {util::print_trace_prefix();fprintf(stderr, format, __VA_ARGS__);util::print_trace_postfix();}}
#define log_trace_(format) {if (util::logging::LogLevel::TRACE >= util::logging::log_level) {util::print_trace_prefix();fprintf(stderr, format);util::print_trace_postfix();}}
#endif

#define log_debug(format, ...) {if (util::logging::LogLevel::DEBUG >= util::logging::log_level) {util::print_debug_prefix();fprintf(stderr, format, __VA_ARGS__);util::print_debug_postfix();}}
#define log_debug_(format) {if (util::logging::LogLevel::DEBUG >= util::logging::log_level) {util::print_debug_prefix();fprintf(stderr, format);util::print_debug_postfix();}}
#define log_info(format, ...) {if (util::logging::LogLevel::INFO >= util::logging::log_level) {util::print_info_prefix();fprintf(stderr, format, __VA_ARGS__);util::print_info_postfix();}}
#define log_info_(format) {if (util::logging::LogLevel::INFO >= util::logging::log_level) {util::print_info_prefix();fprintf(stderr, format);util::print_info_postfix();}}
#define log_warning(format, ...) {if (util::logging::LogLevel::WARNING >= util::logging::log_level) {util::print_warning_prefix();fprintf(stderr, format, __VA_ARGS__);util::print_warning_postfix();}}
#define log_warning_(format) {if (util::logging::LogLevel::WARNING >= util::logging::log_level) {util::print_warning_prefix();fprintf(stderr, format);util::print_warning_postfix();}}
#define log_error(format, ...) {if (util::logging::LogLevel::ERROR >= util::logging::log_level) {util::print_error_prefix();fprintf(stderr, format, __VA_ARGS__);util::print_error_postfix();}}
#define log_error_(format) {if (util::logging::LogLevel::ERROR >= util::logging::log_level) {util::print_error_prefix();fprintf(stderr, format);util::print_error_postfix();}}
#define log_critical(format, ...) {if (util::logging::LogLevel::CRITICAL >= util::logging::log_level) {util::print_critical_prefix();fprintf(stderr, format, __VA_ARGS__);util::print_critical_postfix();}}
#define log_critical_(format) {if (util::logging::LogLevel::CRITICAL >= util::logging::log_level) {util::print_critical_prefix();fprintf(stderr, format);util::print_critical_postfix();}}
#define log_success(format, ...) {if (util::logging::LogLevel::SUCCESS >= util::logging::log_level) {util::print_success_prefix();fprintf(stderr, format, __VA_ARGS__);util::print_success_postfix();}}
#define log_success_(format) {if (util::logging::LogLevel::SUCCESS >= util::logging::log_level) {util::print_success_prefix();fprintf(stderr, format);util::print_success_postfix();}}
// clang-format on
