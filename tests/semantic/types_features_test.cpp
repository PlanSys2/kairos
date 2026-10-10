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

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <set>
#include <string>

#include "semantic/features.hpp"
#include "semantic/model.hpp"

namespace kairos::sem
{
namespace
{

// vehicle - object; car truck - vehicle; amphibian - (either car boat); boat - object
class TypeHierarchyTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    vehicle = h.declare("vehicle", {});
    car = h.declare("car", {});
    truck = h.declare("truck", {});
    boat = h.declare("boat", {});
    amphibian = h.declare("amphibian", {});
    h.add_parent(car, vehicle);
    h.add_parent(truck, vehicle);
    h.add_parent(amphibian, car);
    h.add_parent(amphibian, boat);
    ASSERT_FALSE(h.finalize().has_value());
  }

  TypeHierarchy h;
  TypeId vehicle, car, truck, boat, amphibian;
};

TEST_F(TypeHierarchyTest, ObjectIsTheRoot)
{
  EXPECT_EQ(h.object(), TypeId(0));
  EXPECT_EQ(h[h.object()].name, "object");
  EXPECT_TRUE(h.is_subtype(car, h.object()));
  EXPECT_TRUE(h.is_subtype(h.object(), h.object()));
  EXPECT_FALSE(h.is_subtype(h.object(), car));
}

TEST_F(TypeHierarchyTest, Subtypes)
{
  EXPECT_TRUE(h.is_subtype(car, car));
  EXPECT_TRUE(h.is_subtype(car, vehicle));
  EXPECT_TRUE(h.is_subtype(amphibian, vehicle));  // through car
  EXPECT_TRUE(h.is_subtype(amphibian, boat));     // multiple parents
  EXPECT_FALSE(h.is_subtype(vehicle, car));
  EXPECT_FALSE(h.is_subtype(truck, car));
  EXPECT_FALSE(h.is_subtype(boat, vehicle));
}

TEST_F(TypeHierarchyTest, DeclareIsIdempotentAndFindable)
{
  EXPECT_EQ(h.declare("car", {}), car);
  EXPECT_EQ(h.find("truck"), truck);
  EXPECT_FALSE(h.find("plane").has_value());
  EXPECT_EQ(h.size(), 6u);
}

TEST_F(TypeHierarchyTest, ObjectsFitTheirTypesAndSupertypes)
{
  EXPECT_TRUE(h.fits({car}, {vehicle}));
  EXPECT_TRUE(h.fits({car}, {car}));
  EXPECT_FALSE(h.fits({vehicle}, {car}));  // an object of type vehicle is not a car
  EXPECT_TRUE(h.fits({truck}, {car, truck}));  // either
  EXPECT_TRUE(h.fits({boat, truck}, {vehicle}));  // an object with several types
  EXPECT_FALSE(h.fits({boat}, {vehicle}));
}

TEST_F(TypeHierarchyTest, VariablesOnlyNeedCompatibleTypes)
{
  EXPECT_TRUE(h.compatible({vehicle}, {car}));  // a vehicle variable may hold a car
  EXPECT_TRUE(h.compatible({car}, {vehicle}));
  EXPECT_TRUE(h.compatible({car}, {boat}));     // amphibian is both
  EXPECT_FALSE(h.compatible({truck}, {boat}));  // nothing is both
  EXPECT_FALSE(h.compatible({truck}, {car}));
  EXPECT_TRUE(h.compatible({h.object()}, {truck}));
}

TEST_F(TypeHierarchyTest, UnknownTypesAreAcceptedEverywhere)
{
  EXPECT_TRUE(h.fits({}, {car}));
  EXPECT_TRUE(h.fits({truck}, {}));
  EXPECT_TRUE(h.compatible({}, {boat}));
  EXPECT_EQ(h.to_string({}), "(unknown type)");
}

TEST_F(TypeHierarchyTest, ToString)
{
  EXPECT_EQ(h.to_string({car}), "car");
  EXPECT_EQ(h.to_string({car, boat}), "(either car boat)");
}

TEST(TypeHierarchyCycleTest, CyclesAreDetected)
{
  TypeHierarchy h;
  TypeId a = h.declare("a", {});
  TypeId b = h.declare("b", {});
  TypeId c = h.declare("c", {});
  h.add_parent(a, b);
  h.add_parent(b, c);
  h.add_parent(c, a);
  auto cycle = h.finalize();
  ASSERT_TRUE(cycle.has_value());
  EXPECT_TRUE(*cycle == a || *cycle == b || *cycle == c);
}

TEST(TypeHierarchyCycleTest, SelfParent)
{
  TypeHierarchy h;
  TypeId a = h.declare("a", {});
  h.add_parent(a, a);
  EXPECT_EQ(h.finalize(), a);
}

TEST(TypeHierarchyCycleTest, DeepChainHasNoRecursionLimit)
{
  // Also a memory check: precomputing ancestor sets would need ~200 million entries here
  // (it was killed in the 512 MB CI job)
  TypeHierarchy h;
  TypeId prev = h.object();
  for (int i = 0; i < 20000; ++i) {
    TypeId t = h.declare("t" + std::to_string(i), {});
    h.add_parent(t, prev);
    prev = t;
  }
  EXPECT_FALSE(h.finalize().has_value());
  EXPECT_TRUE(h.is_subtype(prev, *h.find("t0")));
  EXPECT_FALSE(h.is_subtype(*h.find("t0"), prev));
  EXPECT_TRUE(h.compatible({*h.find("t5")}, {prev}));
}

TEST(TypeHierarchyCycleTest, DiamondWithMultipleInheritance)
{
  // d derives from a and b, which both derive from top
  TypeHierarchy h;
  TypeId top = h.declare("top", {});
  TypeId a = h.declare("a", {});
  TypeId b = h.declare("b", {});
  TypeId d = h.declare("d", {});
  h.add_parent(a, top);
  h.add_parent(b, top);
  h.add_parent(d, a);
  h.add_parent(d, b);
  ASSERT_FALSE(h.finalize().has_value());
  EXPECT_TRUE(h.is_subtype(d, top));
  EXPECT_TRUE(h.compatible({a}, {b}));  // d is both
  EXPECT_FALSE(h.is_subtype(a, b));
}

TEST(FeatureSetTest, AddContainsMergeMinus)
{
  FeatureSet a;
  EXPECT_TRUE(a.empty());
  a.add(Feature::DurativeActions);
  a.add(Feature::NumericFluents);
  EXPECT_TRUE(a.contains(Feature::DurativeActions));
  EXPECT_FALSE(a.contains(Feature::Events));
  FeatureSet b;
  b.add(Feature::NumericFluents);
  b.add(Feature::Events);
  FeatureSet merged = a;
  merged.merge(b);
  EXPECT_EQ(merged.list().size(), 3u);
  FeatureSet only_a = a.minus(b);
  EXPECT_EQ(only_a.list(), (std::vector<Feature>{Feature::DurativeActions}));
}

TEST(FeatureSetTest, NamesRoundTripAndAreUnique)
{
  std::set<std::string> names;
  for (uint8_t i = 0; i < static_cast<uint8_t>(Feature::Count); ++i) {
    auto f = static_cast<Feature>(i);
    std::string n(to_string(f));
    EXPECT_TRUE(names.insert(n).second) << n;
    EXPECT_EQ(parse_feature(n), f);
  }
  EXPECT_FALSE(parse_feature("bogus").has_value());
  EXPECT_EQ(to_string(Feature::Count), "count");
}

TEST(FeatureSetTest, NamesInDeclarationOrder)
{
  FeatureSet s;
  s.add(Feature::Events);
  s.add(Feature::Typing);
  EXPECT_EQ(s.names(), (std::vector<std::string>{"typing", "events"}));
}

TEST(FeatureSetTest, CoveringRequirements)
{
  auto neg = covering_requirements(Feature::NegativePreconditions);
  EXPECT_NE(std::find(neg.begin(), neg.end(), ":adl"), neg.end());
  EXPECT_NE(std::find(neg.begin(), neg.end(), ":negative-preconditions"), neg.end());
  auto numeric = covering_requirements(Feature::NumericFluents);
  EXPECT_NE(std::find(numeric.begin(), numeric.end(), ":fluents"), numeric.end());
  EXPECT_TRUE(covering_requirements(Feature::Metric).empty());
  // Every feature but the metric ones and ENHSP's global constraints needs a requirement
  for (uint8_t i = 0; i < static_cast<uint8_t>(Feature::Count); ++i) {
    auto f = static_cast<Feature>(i);
    bool exempt = f == Feature::Metric || f == Feature::MetricTotalTime ||
      f == Feature::MetricFluents || f == Feature::GlobalConstraints;
    EXPECT_EQ(covering_requirements(f).empty(), exempt) << to_string(f);
  }
}

}  // namespace
}  // namespace kairos::sem
