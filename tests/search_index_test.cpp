/// @file
/// Conformance test of the search engines: the same records written and read back
/// through every engine of search_engine.h give the same results.
#include <gtest/gtest.h>

#include <filesystem>

#include "search/search_engine.h"
#include "search/search_rank.h"

using namespace nav;

namespace {

/// Every engine there is; a new engine is added here and must pass the same tests.
constexpr Vrst_search_engine kEngines[] = {Vrst_search_engine::Sqlite};

/// Writes a small region: two places, three streets, house numbers.
/// @param engine Engine.
/// @param dir Folder.
void write_region(Vrst_search_engine engine, const std::filesystem::path& dir) {
  const std::unique_ptr<Vrst_search_index_writer> w = make_search_writer(engine, dir);
  w->add_street({0, Vrst_search_kind::Place, PlaceKind::City, "Dijon", {}, {}, {473220000, 50410000}});
  w->add_street({1, Vrst_search_kind::Place, PlaceKind::Village, "Fontaine", {}, {}, {473400000, 49900000}});
  w->add_street({2, Vrst_search_kind::Street, PlaceKind::City, "Rue de la Libert\xC3\xA9", "Dijon", "21000",
                 {473215000, 50380000}});
  w->add_street({3, Vrst_search_kind::Street, PlaceKind::City, "Rue de Dijon", "Fontaine", {}, {473390000, 49910000}});
  w->add_street({4, Vrst_search_kind::Street, PlaceKind::City, "Rue de la Libert\xC3\xA9", "Fontaine", {},
                 {473410000, 49920000}});
  w->add_address({2, "12", {473216000, 50381000}});
  w->add_address({2, "12bis", {473216500, 50381500}});
  w->add_address({4, "3", {473411000, 49921000}});
  EXPECT_GT(w->finish(42), 0u);
}

}  // namespace

/// Prefix words, house numbers, places first and the build id, for every engine.
TEST(SearchIndex, EnginesGiveTheSameResults) {
  for (const Vrst_search_engine engine : kEngines) {
    const std::filesystem::path dir =
        std::filesystem::temp_directory_path() / ("vrst-search-" + std::to_string(static_cast<int>(engine)));
    std::filesystem::create_directories(dir);
    write_region(engine, dir);
    const std::unique_ptr<Vrst_search_index> index = open_search_index(dir);
    ASSERT_TRUE(index);
    EXPECT_EQ(index->build_id(), 42u);

    std::vector<Vrst_search_result> r = search_index(*index, "12 bis rue de la liberte dijon", std::nullopt, 5);
    ASSERT_FALSE(r.empty());
    EXPECT_EQ(r[0].kind, Vrst_search_kind::Address);
    EXPECT_EQ(r[0].title, "12 bis Rue de la Libert\xC3\xA9");

    r = search_index(*index, "libe", std::nullopt, 5);
    EXPECT_EQ(r.size(), 2u);

    r = search_index(*index, "dijon", std::nullopt, 5);
    ASSERT_EQ(r.size(), 3u);  // the city, its street, "Rue de Dijon"
    EXPECT_EQ(r[0].kind, Vrst_search_kind::Place);

    r = search_index(*index, "3 rue de la liberte", std::nullopt, 5);
    ASSERT_EQ(r.size(), 2u);
    EXPECT_EQ(r[0].detail, "Fontaine");  // the street that has number 3 comes first
    EXPECT_EQ(r[1].detail, "no number 3 - Dijon 21000");

    EXPECT_TRUE(search_index(*index, "zzz", std::nullopt, 5).empty());
    std::filesystem::remove_all(dir);
  }
}
