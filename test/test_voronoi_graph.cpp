#include "vas_planner/voronoi_graph.hpp"

#include <cstdio>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <unistd.h>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

namespace vas_planner
{
namespace
{
using Json = nlohmann::json;

class GraphTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    char path[] = "/tmp/voronoi_graph_test_XXXXXX";
    const int file = mkstemp(path);
    ASSERT_NE(file, -1);
    close(file);
    filename_ = path;
  }

  void TearDown() override {std::remove(filename_.c_str());}

  static Json graphDocument()
  {
    // The branch initially closer to the goal costs 11; the other costs 5.
    // IDs deliberately differ from vector indices and have no ordering.
    return {
      {"format_version", 1}, {"directed", false}, {"map", {{"units", "m"}}},
      {"parameters", {{"robot_radius", 0.2}, {"safety_margin", 0.05}}},
      {"nodes", {
          {{"id", 70}, {"position", {0.0, 0.0}}, {"component", 99}},
          {{"id", 5}, {"position", {2.0, 0.0}}, {"component", 98}},
          {{"id", 800}, {"position", {0.0, 1.0}}, {"component", 97}},
          {{"id", 31}, {"position", {3.0, 0.0}}, {"component", 96}},
          {{"id", 42}, {"position", {10.0, 10.0}}, {"component", 99}}
        }},
      {"edges", {
          {{"source", 70}, {"target", 5}, {"length", 2.0}, {"min_clearance", 0.8}},
          {{"source", 5}, {"target", 31}, {"length", 9.0}, {"min_clearance", 0.8}},
          {{"source", 70}, {"target", 800}, {"length", 1.0}, {"min_clearance", 0.3}},
          {{"source", 800}, {"target", 31}, {"length", 4.0}, {"min_clearance", 0.3}}
        }}
    };
  }

  void write(const Json & document)
  {
    std::ofstream output(filename_);
    output << document.dump();
    ASSERT_TRUE(output.good());
  }

  VoronoiGraph load(const Json & document = graphDocument())
  {
    write(document);
    VoronoiGraph graph;
    graph.load(filename_);
    return graph;
  }

  std::string filename_;
};

TEST_F(GraphTest, LoadsSparseIdsAndMetricMetadata)
{
  const auto graph = load();
  ASSERT_EQ(graph.nodes().size(), 5u);
  ASSERT_EQ(graph.edges().size(), 4u);
  EXPECT_EQ(graph.nodes()[1].id, 5);
  EXPECT_EQ(graph.edges()[0].source, 0u);
  EXPECT_EQ(graph.edges()[0].target, 1u);
  EXPECT_DOUBLE_EQ(graph.robotRadius(), 0.2);
  EXPECT_DOUBLE_EQ(graph.safetyMargin(), 0.05);
  EXPECT_EQ(graph.nearestNode(2.05, 0.0), 1u);
  EXPECT_EQ(graph.nearestNode(9.8, 10.0), 4u);
}

TEST_F(GraphTest, FindsShortestRouteInsteadOfGreedyRoute)
{
  const auto graph = load();
  EXPECT_EQ(graph.findPath(0, 3), (std::vector<size_t>{0, 2, 3}));
}

TEST_F(GraphTest, SupportsReverseUndirectedTraversal)
{
  EXPECT_EQ(load().findPath(3, 0), (std::vector<size_t>{3, 2, 0}));
}

TEST_F(GraphTest, HandlesDisconnectedAndIdenticalEndpoints)
{
  const auto graph = load();
  EXPECT_TRUE(graph.findPath(0, 4).empty());
  EXPECT_EQ(graph.findPath(4, 4), (std::vector<size_t>{4}));
}

TEST_F(GraphTest, BlockedCostmapEdgeUsesAlternativeRoute)
{
  const auto graph = load();
  EXPECT_EQ(graph.findPath(0, 3, 0.0, [](size_t edge) {return edge != 2;}),
    (std::vector<size_t>{0, 1, 3}));
  EXPECT_TRUE(graph.findPath(0, 3, 0.0, [](size_t) {return false;}).empty());
}

TEST_F(GraphTest, StrictClearanceFiltersUnsafeEdges)
{
  const auto graph = load();
  EXPECT_EQ(graph.findPath(0, 3, 0.25), (std::vector<size_t>{0, 2, 3}));
  EXPECT_EQ(graph.findPath(0, 3, 0.3), (std::vector<size_t>{0, 1, 3}));
  EXPECT_TRUE(graph.findPath(0, 3, 0.8).empty());
}

TEST_F(GraphTest, ScalesHeuristicWhenCostsAreShorterThanGeometricDistances)
{
  auto document = graphDocument();
  document["nodes"][2]["position"] = {100.0, 0.0};
  document["edges"][3]["length"] = 1.0;
  // An unscaled geometric heuristic would prematurely choose the cost-11 route.
  EXPECT_EQ(load(document).findPath(0, 3), (std::vector<size_t>{0, 2, 3}));
}

TEST_F(GraphTest, AcceptsIsolatedNodesAndOptionalParameters)
{
  auto document = graphDocument();
  document["edges"] = Json::array();
  document.erase("parameters");
  const auto graph = load(document);
  EXPECT_TRUE(graph.edges().empty());
  EXPECT_TRUE(graph.findPath(0, 3).empty());
  EXPECT_DOUBLE_EQ(graph.robotRadius(), 0.0);
  EXPECT_DOUBLE_EQ(graph.safetyMargin(), 0.0);
}

TEST_F(GraphTest, RejectsPixelCoordinatesWithActionableMessage)
{
  auto document = graphDocument();
  document["map"]["units"] = "px";
  write(document);
  VoronoiGraph graph;
  try {
    graph.load(filename_);
    FAIL() << "Expected pixel graph rejection";
  } catch (const std::runtime_error & error) {
    EXPECT_NE(std::string(error.what()).find("--resolution"), std::string::npos);
  }
}

TEST_F(GraphTest, RejectsInvalidFormatAndContainerTypes)
{
  std::vector<Json> documents;
  auto document = graphDocument();
  document["format_version"] = 2;
  documents.push_back(document);
  document = graphDocument();
  document["directed"] = true;
  documents.push_back(document);
  document = graphDocument();
  document["nodes"] = Json::array();
  documents.push_back(document);
  document = graphDocument();
  document["edges"] = Json::object();
  documents.push_back(document);
  document = graphDocument();
  document.erase("map");
  documents.push_back(document);
  for (const auto & invalid : documents) {
    write(invalid);
    VoronoiGraph graph;
    EXPECT_THROW(graph.load(filename_), std::runtime_error);
  }
}

TEST_F(GraphTest, RejectsInvalidIdsPositionsAndReferences)
{
  std::vector<Json> documents;
  for (const Json & invalid_id : std::vector<Json>{-1, 1.5, "7", true,
      std::numeric_limits<uint64_t>::max()})
  {
    auto document = graphDocument();
    document["nodes"][0]["id"] = invalid_id;
    documents.push_back(document);
  }
  auto document = graphDocument();
  document["nodes"][1]["id"] = 70;
  documents.push_back(document);
  document = graphDocument();
  document["nodes"][0]["position"] = {1.0};
  documents.push_back(document);
  document = graphDocument();
  document["nodes"][0]["position"][0] = "bad";
  documents.push_back(document);
  document = graphDocument();
  document["nodes"][0]["position"][0] = nullptr;
  documents.push_back(document);
  document = graphDocument();
  document["edges"][0]["source"] = 999999;
  documents.push_back(document);
  for (const auto & invalid : documents) {
    write(invalid);
    VoronoiGraph graph;
    EXPECT_THROW(graph.load(filename_), std::runtime_error);
  }
}

TEST_F(GraphTest, RejectsInvalidLengthsClearancesAndParameters)
{
  std::vector<Json> documents;
  for (double length : {0.0, -1.0}) {
    auto document = graphDocument();
    document["edges"][0]["length"] = length;
    documents.push_back(document);
  }
  auto document = graphDocument();
  document["edges"][0]["min_clearance"] = -0.1;
  documents.push_back(document);
  document = graphDocument();
  document["parameters"]["robot_radius"] = -0.1;
  documents.push_back(document);
  document = graphDocument();
  document["parameters"]["safety_margin"] = "bad";
  documents.push_back(document);
  document = graphDocument();
  document["parameters"] = nullptr;
  documents.push_back(document);
  for (const auto & invalid : documents) {
    write(invalid);
    VoronoiGraph graph;
    EXPECT_THROW(graph.load(filename_), std::runtime_error);
  }
}

TEST_F(GraphTest, RejectsMalformedJsonAndMissingFile)
{
  VoronoiGraph graph;
  EXPECT_THROW(graph.load(filename_ + ".missing"), std::runtime_error);
  {
    std::ofstream output(filename_);
    output << "{bad json}";
  }
  EXPECT_THROW(graph.load(filename_), std::runtime_error);
}

TEST_F(GraphTest, FailedReloadPreservesPreviousGraph)
{
  auto graph = load();
  auto document = graphDocument();
  document["edges"][2]["length"] = -1.0;
  write(document);
  EXPECT_THROW(graph.load(filename_), std::runtime_error);
  ASSERT_EQ(graph.edges().size(), 4u);
  EXPECT_EQ(graph.findPath(0, 3), (std::vector<size_t>{0, 2, 3}));
}

TEST_F(GraphTest, RejectsInvalidSearchInputs)
{
  const auto graph = load();
  EXPECT_THROW(graph.findPath(99, 0), std::out_of_range);
  EXPECT_THROW(graph.findPath(0, 99), std::out_of_range);
  EXPECT_THROW(graph.findPath(0, 3, -1.0), std::invalid_argument);
  EXPECT_THROW(graph.findPath(0, 3, std::numeric_limits<double>::infinity()),
    std::invalid_argument);
  EXPECT_THROW(graph.nearestNode(std::numeric_limits<double>::quiet_NaN(), 0.0),
    std::invalid_argument);
  const VoronoiGraph empty;
  EXPECT_THROW(empty.nearestNode(0.0, 0.0), std::runtime_error);
}

}  // namespace
}  // namespace vas_planner
