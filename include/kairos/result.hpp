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

/// \file
/// \brief Result<T>: a value or the errors that prevented it, plus any warnings.

#ifndef KAIROS__RESULT_HPP_
#define KAIROS__RESULT_HPP_

#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include "kairos/diagnostics.hpp"

namespace kairos
{

/// @brief Thrown by Result::value() on a failed result (a programming error).
class BadResultAccess : public std::logic_error
{
public:
  BadResultAccess()
  : std::logic_error("kairos::Result: value() called on a failed result") {}
};

/**
 * @brief Either a value or at least one error diagnostic.
 *
 * Warnings and notes may come with a successful value too. A failure always holds an error:
 * failure() adds "core.missing_error" if the caller passed none.
 */
template<class T>
class [[nodiscard]] Result
{
public:
  static Result success(T value, std::vector<Diagnostic> diagnostics = {})
  {
    Result r;
    r.value_.emplace(std::move(value));
    r.diagnostics_ = std::move(diagnostics);
    return r;
  }

  static Result failure(std::vector<Diagnostic> diagnostics)
  {
    Result r;
    r.diagnostics_ = std::move(diagnostics);
    if (!has_errors(r.diagnostics_)) {
      r.diagnostics_.push_back(
        Diagnostic::error("core.missing_error", "operation failed without reporting an error"));
    }
    return r;
  }

  static Result failure(Diagnostic diagnostic)
  {
    return failure(std::vector<Diagnostic>{std::move(diagnostic)});
  }

  bool ok() const noexcept {return value_.has_value();}
  explicit operator bool() const noexcept {return ok();}

  T & value() &
  {
    check();
    return *value_;
  }

  const T & value() const &
  {
    check();
    return *value_;
  }

  T && value() &&
  {
    check();
    return std::move(*value_);
  }

  T & operator*() & {return value();}
  const T & operator*() const & {return value();}
  T * operator->() {return &value();}
  const T * operator->() const {return &value();}

  const std::vector<Diagnostic> & diagnostics() const noexcept {return diagnostics_;}

  /// @brief Moves the diagnostics out, e.g. to forward them in another Result.
  std::vector<Diagnostic> take_diagnostics() noexcept {return std::move(diagnostics_);}

private:
  Result() = default;

  void check() const
  {
    if (!value_) {
      throw BadResultAccess();
    }
  }

  std::optional<T> value_;
  std::vector<Diagnostic> diagnostics_;
};

}  // namespace kairos

#endif  // KAIROS__RESULT_HPP_
