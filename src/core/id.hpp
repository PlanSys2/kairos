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
/// \brief Strongly typed integer ids and vectors indexed by them.

#ifndef KAIROS__CORE__ID_HPP_
#define KAIROS__CORE__ID_HPP_

#include <compare>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace kairos
{

/**
 * @brief An index tagged with the kind of entity it identifies.
 *
 * Id<AtomTag> and Id<ActionTag> do not convert into each other. A default constructed id is
 * invalid.
 */
template<class Tag, class Rep = uint32_t>
class Id
{
public:
  using rep_type = Rep;
  static constexpr Rep invalid_value = std::numeric_limits<Rep>::max();

  constexpr Id() noexcept = default;
  constexpr explicit Id(Rep value) noexcept
  : value_(value) {}

  static constexpr Id invalid() noexcept {return Id();}

  constexpr Rep value() const noexcept {return value_;}
  constexpr bool valid() const noexcept {return value_ != invalid_value;}
  constexpr size_t index() const noexcept {return static_cast<size_t>(value_);}

  constexpr auto operator<=>(const Id &) const noexcept = default;

private:
  Rep value_ = invalid_value;
};

/// @brief A std::vector indexed by an Id type instead of size_t.
template<class IdT, class T>
class IdVector
{
public:
  using value_type = T;
  using iterator = typename std::vector<T>::iterator;
  using const_iterator = typename std::vector<T>::const_iterator;

  IdVector() = default;
  explicit IdVector(size_t n, const T & value = T())
  : data_(n, value) {}

  /// @brief Appends and returns the id of the new element.
  IdT push_back(T value)
  {
    if (data_.size() >= static_cast<size_t>(IdT::invalid_value)) {
      throw std::length_error("kairos::IdVector: id space exhausted");
    }
    data_.push_back(std::move(value));
    return IdT(static_cast<typename IdT::rep_type>(data_.size() - 1));
  }

  template<class ... Args>
  IdT emplace_back(Args && ... args)
  {
    return push_back(T(std::forward<Args>(args)...));
  }

  T & operator[](IdT id) {return data_[id.index()];}
  const T & operator[](IdT id) const {return data_[id.index()];}

  /// @brief Checked access; throws std::out_of_range on invalid or out of range ids.
  T & at(IdT id) {return data_.at(checked(id));}
  const T & at(IdT id) const {return data_.at(checked(id));}

  bool contains(IdT id) const noexcept {return id.valid() && id.index() < data_.size();}

  size_t size() const noexcept {return data_.size();}
  bool empty() const noexcept {return data_.empty();}
  void reserve(size_t n) {data_.reserve(n);}
  void clear() noexcept {data_.clear();}

  iterator begin() noexcept {return data_.begin();}
  iterator end() noexcept {return data_.end();}
  const_iterator begin() const noexcept {return data_.begin();}
  const_iterator end() const noexcept {return data_.end();}

  /// @brief Ids of all elements, in order.
  std::vector<IdT> ids() const
  {
    std::vector<IdT> result;
    result.reserve(data_.size());
    for (size_t i = 0; i < data_.size(); ++i) {
      result.emplace_back(static_cast<typename IdT::rep_type>(i));
    }
    return result;
  }

private:
  static size_t checked(IdT id)
  {
    if (!id.valid()) {
      throw std::out_of_range("kairos::IdVector: invalid id");
    }
    return id.index();
  }

  std::vector<T> data_;
};

}  // namespace kairos

template<class Tag, class Rep>
struct std::hash<kairos::Id<Tag, Rep>>
{
  size_t operator()(const kairos::Id<Tag, Rep> & id) const noexcept
  {
    return std::hash<Rep>{}(id.value());
  }
};

#endif  // KAIROS__CORE__ID_HPP_
