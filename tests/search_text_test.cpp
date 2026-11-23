/// @file
/// Tests of the address search text rules (search_text.h).
#include <gtest/gtest.h>

#include "search/search_text.h"

using namespace nav;

/// Accents, ligatures, case and punctuation fold to plain lowercase words.
TEST(SearchText, NormalizeFoldsLatin) {
  EXPECT_EQ(normalize("Rue de l'\xC3\x89glise"), "rue de l eglise");            // E acute
  EXPECT_EQ(normalize("C\xC5\x93ur  -  Stra\xC3\x9F" "e"), "coeur strasse");   // oe, sharp s
  EXPECT_EQ(normalize("Place du 8 Mai 1945"), "place du 8 mai 1945");
  EXPECT_EQ(normalize("l\xE2\x80\x99H\xC3\xB4tel"), "l hotel");                // curly apostrophe
  EXPECT_EQ(normalize("  "), "");
}

/// Letters outside Latin are kept as they are.
TEST(SearchText, NormalizeKeepsOtherScripts) {
  EXPECT_EQ(normalize("\xD0\x9C\xD0\xB8\xD1\x80"), "\xD0\x9C\xD0\xB8\xD1\x80");
}

/// House number keys and their display form.
TEST(SearchText, NumberKeyAndDisplay) {
  EXPECT_EQ(number_key("12 bis"), "12bis");
  EXPECT_EQ(number_key("12B"), "12b");
  EXPECT_EQ(number_key("A"), "");
  EXPECT_EQ(number_display("12bis"), "12 bis");
  EXPECT_EQ(number_display("12b"), "12b");
}

/// The number is found before or after the street, with its suffix.
TEST(SearchText, ParseQueryNumber) {
  Vrst_search_query q = parse_query("12 bis rue de la Liberte, Dijon");
  EXPECT_EQ(q.number, "12bis");
  EXPECT_EQ(q.terms, (std::vector<std::string>{"rue", "de", "la", "liberte", "dijon"}));
  q = parse_query("rue de la paix 7");
  EXPECT_EQ(q.number, "7");
  q = parse_query("12 r de la paix");
  EXPECT_EQ(q.number, "12");
  EXPECT_EQ(q.terms.front(), "rue");
}

/// Postcodes stay words; a lone number is a word; abbreviations expand.
TEST(SearchText, ParseQueryWords) {
  Vrst_search_query q = parse_query("Dijon 21000");
  EXPECT_TRUE(q.number.empty());
  EXPECT_EQ(q.terms, (std::vector<std::string>{"dijon", "21000"}));
  q = parse_query("12");
  EXPECT_TRUE(q.number.empty());
  EXPECT_EQ(q.terms, (std::vector<std::string>{"12"}));
  q = parse_query("bd st michel");
  EXPECT_EQ(q.terms, (std::vector<std::string>{"boulevard", "saint", "michel"}));
  EXPECT_TRUE(parse_query("").terms.empty());
}
