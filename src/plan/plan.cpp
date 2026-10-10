// Copyright 2026 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "kairos/plan.hpp"

#include <algorithm>
#include <cctype>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

namespace kairos
{

namespace
{

constexpr std::string_view kPopfMarker = ";;;; Solution Found";

std::string lower(std::string_view s)
{
  std::string out(s);
  for (auto & c : out) {
    if (c >= 'A' && c <= 'Z') {
      c = static_cast<char>(c - 'A' + 'a');
    }
  }
  return out;
}

bool is_space(char c) {return c == ' ' || c == '\t' || c == '\r' || c == '\v' || c == '\f';}

std::string fixed(double value, int precision)
{
  // Avoid "-0.000"
  if (std::fabs(value) < 0.5 * std::pow(10.0, -precision)) {
    value = 0.0;
  }
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), "%.*f", precision, value);
  return buffer;
}

std::string json_number(double value)
{
  char buffer[64];
  auto [ptr, ec] = std::to_chars(buffer, buffer + sizeof(buffer), value);
  return ec == std::errc() ? std::string(buffer, ptr) : std::string("0");
}

std::string json_string(const std::string & s)
{
  std::string out = "\"";
  for (char c : s) {
    if (c == '"' || c == '\\') {
      out += '\\';
    }
    out += c;
  }
  return out + "\"";
}

// Reads one line of a plan; the cursor is a position in the line
class LineReader
{
public:
  LineReader(std::string_view line, uint32_t number, std::string_view file)
  : line_(line), number_(number), file_(file) {}

  void skip_spaces()
  {
    while (pos_ < line_.size() && is_space(line_[pos_])) {
      ++pos_;
    }
  }

  bool at_end()
  {
    skip_spaces();
    return pos_ >= line_.size() || line_[pos_] == ';';
  }

  char peek()
  {
    skip_spaces();
    return pos_ < line_.size() ? line_[pos_] : '\0';
  }

  void consume() {++pos_;}

  // A number such as 12, 0.5, 1e-3
  std::optional<double> number()
  {
    skip_spaces();
    size_t start = pos_;
    while (pos_ < line_.size() &&
      (std::isdigit(static_cast<unsigned char>(line_[pos_])) || line_[pos_] == '.' ||
      line_[pos_] == 'e' || line_[pos_] == 'E' || line_[pos_] == '-' || line_[pos_] == '+'))
    {
      ++pos_;
    }
    double value = 0.0;
    auto [ptr, ec] = std::from_chars(line_.data() + start, line_.data() + pos_, value);
    if (start == pos_ || ec != std::errc() || ptr != line_.data() + pos_ || !std::isfinite(value)) {
      pos_ = start;
      return std::nullopt;
    }
    return value;
  }

  // A name inside an action: anything up to space or parenthesis
  std::string_view word()
  {
    skip_spaces();
    size_t start = pos_;
    while (pos_ < line_.size() && !is_space(line_[pos_]) && line_[pos_] != '(' &&
      line_[pos_] != ')' && line_[pos_] != ';')
    {
      ++pos_;
    }
    return line_.substr(start, pos_ - start);
  }

  Diagnostic error(const std::string & code, const std::string & message) const
  {
    return error_at(code, message, pos_ + 1);
  }

  Diagnostic error_at(const std::string & code, const std::string & message, size_t column) const
  {
    return Diagnostic::error(code, message,
        SourceLocation{std::string(file_), number_, static_cast<uint32_t>(column)});
  }

  // Column of the next non-space character
  size_t next_column()
  {
    skip_spaces();
    return pos_ + 1;
  }

  size_t column() const {return pos_ + 1;}

private:
  std::string_view line_;
  uint32_t number_;
  std::string_view file_;
  size_t pos_ = 0;
};

}  // namespace

std::string PlanStep::action() const
{
  std::string out = "(" + name;
  for (const auto & a : args) {
    out += " " + a;
  }
  return out + ")";
}

double Plan::makespan() const
{
  if (!temporal) {
    return static_cast<double>(steps.size());
  }
  double end = 0.0;
  for (const auto & s : steps) {
    end = std::max(end, s.time + s.duration);
  }
  return end;
}

std::string to_string(const Plan & plan, PlanFormat format, int precision)
{
  precision = std::clamp(precision, 0, 9);
  std::string out;
  switch (format) {
    case PlanFormat::Ipc:
      if (plan.cost) {
        out += "; cost = " + json_number(*plan.cost) + "\n";
      }
      if (plan.temporal) {
        out += "; makespan = " + fixed(plan.makespan(), precision) + "\n";
        for (const auto & s : plan.steps) {
          out += fixed(s.time, precision) + ": " + s.action() + " [" +
            fixed(s.duration, precision) + "]\n";
        }
      } else {
        for (const auto & s : plan.steps) {
          out += s.action() + "\n";
        }
      }
      return out;
    case PlanFormat::Popf:
      // The PlanSys2 POPF plugin reads lines after the marker; no blank lines allowed
      out += std::string(kPopfMarker) + "\n";
      if (plan.cost) {
        out += "; Cost: " + fixed(*plan.cost, precision) + "\n";
      }
      for (size_t i = 0; i < plan.steps.size(); ++i) {
        const auto & s = plan.steps[i];
        double time = plan.temporal ? s.time : static_cast<double>(i);
        out += fixed(time, precision) + ": " + s.action() + "  [" +
          fixed(s.duration, precision) + "]\n";
      }
      return out;
    case PlanFormat::Json: {
        out += "{\"temporal\":" + std::string(plan.temporal ? "true" : "false");
        out += ",\"cost\":" + (plan.cost ? json_number(*plan.cost) : std::string("null"));
        out += ",\"makespan\":" + json_number(plan.makespan());
        out += ",\"steps\":[";
        for (size_t i = 0; i < plan.steps.size(); ++i) {
          const auto & s = plan.steps[i];
          out += i == 0 ? "" : ",";
          out += "{\"time\":" + json_number(s.time) + ",\"duration\":" +
            json_number(s.duration) + ",\"action\":" + json_string(s.action()) + "}";
        }
        out += "]}\n";
        return out;
      }
  }
  return out;
}

Result<Plan> parse_plan(std::string_view text, std::string_view filename)
{
  // POPF/OPTIC stdout: only what follows the last solution marker
  uint32_t first_line = 1;
  if (size_t marker = text.rfind(kPopfMarker); marker != std::string_view::npos) {
    first_line += static_cast<uint32_t>(std::count(text.begin(), text.begin() +
      static_cast<std::ptrdiff_t>(marker), '\n'));
    text = text.substr(marker);
  }

  Plan plan;
  std::vector<Diagnostic> errors;
  bool any_timed = false;
  bool any_untimed = false;
  uint32_t line_number = first_line;
  size_t pos = 0;
  while (pos <= text.size() && errors.size() < 50) {
    size_t end = text.find('\n', pos);
    if (end == std::string_view::npos) {
      end = text.size();
    }
    std::string_view line = text.substr(pos, end - pos);
    LineReader r(line, line_number, filename);
    pos = end + 1;
    ++line_number;
    if (r.at_end()) {
      continue;
    }

    PlanStep step;
    bool timed = false;
    if (r.peek() != '(') {
      // "t:" or "n:" prefix
      size_t time_column = r.next_column();
      auto t = r.number();
      if (!t) {
        errors.push_back(r.error("plan.syntax", "expected '(' or a time followed by ':'"));
        continue;
      }
      if (r.peek() != ':') {
        errors.push_back(r.error("plan.syntax", "expected ':' after the time"));
        continue;
      }
      r.consume();
      if (*t < 0) {
        errors.push_back(r.error_at("plan.negative_time", "negative time", time_column));
        continue;
      }
      step.time = *t;
      timed = true;
    }
    if (r.peek() != '(') {
      errors.push_back(r.error("plan.syntax", "expected '(' starting an action"));
      continue;
    }
    r.consume();
    std::string_view name = r.word();
    if (name.empty()) {
      errors.push_back(r.error("plan.syntax", "expected an action name"));
      continue;
    }
    step.name = lower(name);
    bool closed = false;
    bool bad = false;
    while (!r.at_end()) {
      char c = r.peek();
      if (c == ')') {
        r.consume();
        closed = true;
        break;
      }
      if (c == '(') {
        errors.push_back(r.error("plan.syntax", "nested parentheses in an action"));
        bad = true;
        break;
      }
      step.args.push_back(lower(r.word()));
    }
    if (bad) {
      continue;
    }
    if (!closed) {
      errors.push_back(r.error("plan.syntax", "missing ')' closing the action"));
      continue;
    }
    if (r.peek() == '[') {
      r.consume();
      size_t duration_column = r.next_column();
      auto d = r.number();
      if (!d || r.peek() != ']') {
        errors.push_back(r.error("plan.syntax", "expected a duration and ']'"));
        continue;
      }
      r.consume();
      if (*d < 0) {
        errors.push_back(
          r.error_at("plan.negative_duration", "negative duration", duration_column));
        continue;
      }
      step.duration = *d;
      timed = true;
    }
    if (!r.at_end()) {
      errors.push_back(r.error("plan.syntax", "unexpected text after the action"));
      continue;
    }
    any_timed = any_timed || timed;
    any_untimed = any_untimed || !timed;
    plan.steps.push_back(std::move(step));
  }
  if (!errors.empty()) {
    return Result<Plan>::failure(std::move(errors));
  }

  // "n: (a)" without durations is still sequential when the times are 0, 1, 2...
  bool numbered = any_timed && !any_untimed;
  if (numbered) {
    bool has_duration = std::any_of(plan.steps.begin(), plan.steps.end(),
        [](const PlanStep & s) {return s.duration > 0;});
    bool counting = true;
    for (size_t i = 0; i < plan.steps.size(); ++i) {
      counting = counting && plan.steps[i].time == static_cast<double>(i);
    }
    plan.temporal = has_duration || !counting;
  }
  if (!plan.temporal) {
    for (size_t i = 0; i < plan.steps.size(); ++i) {
      plan.steps[i].time = static_cast<double>(i);
    }
  }
  std::stable_sort(plan.steps.begin(), plan.steps.end(),
    [](const PlanStep & a, const PlanStep & b) {return a.time < b.time;});
  std::vector<Diagnostic> warnings;
  if (any_timed && any_untimed) {
    warnings.push_back(
      Diagnostic::warning(
        "plan.mixed_lines", "some steps have times and others do not; read as sequential",
        SourceLocation{std::string(filename), first_line, 0}));
  }
  return Result<Plan>::success(std::move(plan), std::move(warnings));
}

}  // namespace kairos
