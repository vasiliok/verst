/// @file
/// The SQLite FTS5 engine of the address search. Documentation is in search_sqlite.h.
#include "search/search_sqlite.h"

#include <sqlite3.h>

#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>

#include "search/search_text.h"

namespace nav {

namespace fs = std::filesystem;

namespace {

/// Closes a database handle (unique_ptr deleter).
class Vrst_db_closer {
 public:
  /// Closes it; statements must be finalized before.
  /// @param db Handle.
  void operator()(sqlite3* db) const { sqlite3_close_v2(db); }
};

/// Finalizes a prepared statement (unique_ptr deleter).
class Vrst_stmt_finalizer {
 public:
  /// Finalizes it.
  /// @param s Statement.
  void operator()(sqlite3_stmt* s) const { sqlite3_finalize(s); }
};

/// Owned database handle.
using Vrst_db = std::unique_ptr<sqlite3, Vrst_db_closer>;
/// Owned prepared statement.
using Vrst_stmt = std::unique_ptr<sqlite3_stmt, Vrst_stmt_finalizer>;

/// Throws the database's last error.
/// @param db Database.
/// @param what What was being done.
[[noreturn]] void fail(sqlite3& db, std::string_view what) {
  throw std::runtime_error("search.db: " + std::string(what) + ": " + sqlite3_errmsg(&db));
}

/// Opens a database.
/// @param name File name or URI.
/// @param flags SQLITE_OPEN_* flags.
/// @return Handle.
/// @throws std::runtime_error when it cannot be opened.
Vrst_db open_db(const std::string& name, int flags) {
  sqlite3* raw = nullptr;  // C API out-parameter, owned at once
  const int rc = sqlite3_open_v2(name.c_str(), &raw, flags, nullptr);
  Vrst_db db(raw);
  if (rc != SQLITE_OK) {
    if (!db) throw std::runtime_error("search.db: out of memory opening " + name);
    fail(*db, "open " + name);
  }
  return db;
}

/// Runs SQL that returns no rows.
/// @param db Database.
/// @param sql Statements.
void exec(sqlite3& db, const char* sql) {
  if (sqlite3_exec(&db, sql, nullptr, nullptr, nullptr) != SQLITE_OK) fail(db, sql);
}

/// Prepares one statement.
/// @param db Database.
/// @param sql Statement.
/// @return Statement.
Vrst_stmt prepare(sqlite3& db, std::string_view sql) {
  sqlite3_stmt* raw = nullptr;  // C API out-parameter, owned at once
  if (sqlite3_prepare_v2(&db, sql.data(), static_cast<int>(sql.size()), &raw, nullptr) != SQLITE_OK)
    fail(db, "prepare " + std::string(sql));
  return Vrst_stmt(raw);
}

/// Binds text that stays alive until the statement is reset.
/// @param s Statement.
/// @param i Parameter index, from 1.
/// @param text Text.
void bind_text(sqlite3_stmt& s, int i, std::string_view text) {
  // A null destructor is SQLITE_STATIC: SQLite does not copy the text.
  sqlite3_bind_text(&s, i, text.data(), static_cast<int>(text.size()), nullptr);
}

/// Runs a statement that returns no rows and resets it for the next use.
/// @param db Database.
/// @param s Statement.
void run(sqlite3& db, sqlite3_stmt& s) {
  if (sqlite3_step(&s) != SQLITE_DONE) fail(db, "insert");
  sqlite3_reset(&s);
  sqlite3_clear_bindings(&s);
}

/// Text of a result column.
/// @param s Statement on a row.
/// @param i Column index, from 0.
/// @return Text; empty for NULL.
std::string column_text(sqlite3_stmt& s, int i) {
  const unsigned char* t = sqlite3_column_text(&s, i);
  // SQLite returns UTF-8 as unsigned char; this is the one place it becomes char.
  return t ? std::string(reinterpret_cast<const char*>(t), static_cast<Vrst_size>(sqlite3_column_bytes(&s, i)))
           : std::string();
}

/// Writes search.db; see search_sqlite.h for the tables.
class Vrst_sqlite_writer final : public Vrst_search_index_writer {
 public:
  /// Creates the file and its tables and starts one transaction for all rows.
  /// @param dir Folder.
  explicit Vrst_sqlite_writer(const fs::path& dir);
  void add_street(const Vrst_search_street& s) override;     ///< @copydoc Vrst_search_index_writer::add_street
  void add_address(const Vrst_search_address& a) override;   ///< @copydoc Vrst_search_index_writer::add_address
  Vrst_uint64 finish(Vrst_uint64 build_id) override;          ///< @copydoc Vrst_search_index_writer::finish

 private:
  /// The database file.
  fs::path m_file;
  /// Open database; closed by finish().
  Vrst_db m_db;
  /// INSERT into street; declared after m_db so it is finalized first.
  Vrst_stmt m_street;
  /// INSERT into fts.
  Vrst_stmt m_fts;
  /// INSERT OR IGNORE into addr: a number given twice (a node and a building) is kept once.
  Vrst_stmt m_addr;
};

Vrst_sqlite_writer::Vrst_sqlite_writer(const fs::path& dir) : m_file(dir / kSqliteSearchFile) {
  std::error_code ec;
  fs::remove(m_file, ec);
  m_db = open_db(m_file.string(), SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE);
  // Written once in one go and renamed into place by the converter: no journal needed.
  exec(*m_db,
       "PRAGMA journal_mode=OFF; PRAGMA synchronous=OFF; PRAGMA locking_mode=EXCLUSIVE;"
       "CREATE TABLE meta(key TEXT PRIMARY KEY, value) WITHOUT ROWID;"
       "CREATE TABLE street(id INTEGER PRIMARY KEY, kind INTEGER, place INTEGER, name TEXT, city TEXT,"
       " postcode TEXT, lat INTEGER, lon INTEGER);"
       "CREATE TABLE addr(street_id INTEGER, number TEXT, lat INTEGER, lon INTEGER,"
       " PRIMARY KEY(street_id, number)) WITHOUT ROWID;"
       "CREATE VIRTUAL TABLE fts USING fts5(text, content='', detail=none, prefix='2 3');"
       "BEGIN;");
  m_street = prepare(*m_db, "INSERT INTO street VALUES(?1, ?2, ?3, ?4, ?5, ?6, ?7, ?8)");
  m_fts = prepare(*m_db, "INSERT INTO fts(rowid, text) VALUES(?1, ?2)");
  m_addr = prepare(*m_db, "INSERT OR IGNORE INTO addr VALUES(?1, ?2, ?3, ?4)");
}

void Vrst_sqlite_writer::add_street(const Vrst_search_street& s) {
  sqlite3_stmt& st = *m_street;
  sqlite3_bind_int64(&st, 1, s.id);
  sqlite3_bind_int(&st, 2, static_cast<int>(s.kind));
  sqlite3_bind_int(&st, 3, static_cast<int>(s.place));
  bind_text(st, 4, s.name);
  bind_text(st, 5, s.city);
  bind_text(st, 6, s.postcode);
  sqlite3_bind_int(&st, 7, s.pos.lat_e7);
  sqlite3_bind_int(&st, 8, s.pos.lon_e7);
  run(*m_db, st);
  const std::string text = normalize(s.name + " " + s.city + " " + s.postcode);
  sqlite3_bind_int64(m_fts.get(), 1, s.id);
  bind_text(*m_fts, 2, text);
  run(*m_db, *m_fts);
}

void Vrst_sqlite_writer::add_address(const Vrst_search_address& a) {
  sqlite3_stmt& st = *m_addr;
  sqlite3_bind_int64(&st, 1, a.street_id);
  bind_text(st, 2, a.number);
  sqlite3_bind_int(&st, 3, a.pos.lat_e7);
  sqlite3_bind_int(&st, 4, a.pos.lon_e7);
  run(*m_db, st);
}

Vrst_uint64 Vrst_sqlite_writer::finish(Vrst_uint64 build_id) {
  Vrst_stmt meta = prepare(*m_db, "INSERT INTO meta VALUES('build_id', ?1)");
  sqlite3_bind_int64(meta.get(), 1, static_cast<sqlite3_int64>(build_id));
  run(*m_db, *meta);
  meta.reset();
  m_street.reset();
  m_fts.reset();
  m_addr.reset();
  exec(*m_db, "COMMIT; INSERT INTO fts(fts) VALUES('optimize'); VACUUM;");
  m_db.reset();
  return fs::file_size(m_file);
}

/// Reads search.db.
class Vrst_sqlite_index final : public Vrst_search_index {
 public:
  /// Opens the file read-only and reads its build id.
  /// @param file search.db.
  explicit Vrst_sqlite_index(const fs::path& file);
  Vrst_uint64 build_id() const override { return m_build_id; }  ///< @copydoc Vrst_search_index::build_id
  /// @copydoc Vrst_search_index::find_streets
  std::vector<Vrst_search_street> find_streets(std::span<const std::string> terms, Vrst_size max) const override;
  /// @copydoc Vrst_search_index::find_number
  std::optional<Vrst_coord> find_number(Vrst_uint32 street_id, std::string_view number) const override;

 private:
  /// Open database.
  Vrst_db m_db;
  /// meta build_id.
  Vrst_uint64 m_build_id = 0;
};

Vrst_sqlite_index::Vrst_sqlite_index(const fs::path& file) {
  // immutable=1: the file never changes while open, so SQLite takes no locks.
  std::string uri = "file:";
  for (const char c : file.generic_string()) {
    if (c == '%' || c == '?' || c == '#') {
      static constexpr char kHex[] = "0123456789abcdef";
      uri += '%';
      uri += kHex[static_cast<unsigned char>(c) >> 4];
      uri += kHex[static_cast<unsigned char>(c) & 0xF];
    } else {
      uri += c;
    }
  }
  uri += "?immutable=1";
  m_db = open_db(uri, SQLITE_OPEN_READONLY | SQLITE_OPEN_URI);
  const Vrst_stmt s = prepare(*m_db, "SELECT value FROM meta WHERE key = 'build_id'");
  if (sqlite3_step(s.get()) != SQLITE_ROW) fail(*m_db, "no build id");
  m_build_id = static_cast<Vrst_uint64>(sqlite3_column_int64(s.get(), 0));
}

std::vector<Vrst_search_street> Vrst_sqlite_index::find_streets(std::span<const std::string> terms,
                                                                 Vrst_size max) const {
  std::vector<Vrst_search_street> out;
  if (terms.empty() || max == 0) return out;
  // Every term as a prefix, all required: "liber"* "dijon"*. Normalized words hold no quotes.
  std::string match;
  for (const std::string& t : terms) match += (match.empty() ? "\"" : " \"") + t + "\"*";
  const Vrst_stmt s = prepare(*m_db,
                              "SELECT s.id, s.kind, s.place, s.name, s.city, s.postcode, s.lat, s.lon"
                              " FROM fts JOIN street s ON s.id = fts.rowid WHERE fts MATCH ?1 LIMIT ?2");
  bind_text(*s, 1, match);
  sqlite3_bind_int64(s.get(), 2, static_cast<sqlite3_int64>(max));
  int rc = SQLITE_OK;
  while ((rc = sqlite3_step(s.get())) == SQLITE_ROW) {
    sqlite3_stmt& r = *s;
    out.push_back({static_cast<Vrst_uint32>(sqlite3_column_int64(&r, 0)),
                   static_cast<Vrst_search_kind>(sqlite3_column_int(&r, 1)),
                   static_cast<PlaceKind>(sqlite3_column_int(&r, 2)), column_text(r, 3), column_text(r, 4),
                   column_text(r, 5), Vrst_coord{sqlite3_column_int(&r, 6), sqlite3_column_int(&r, 7)}});
  }
  if (rc != SQLITE_DONE) fail(*m_db, "find streets");
  return out;
}

std::optional<Vrst_coord> Vrst_sqlite_index::find_number(Vrst_uint32 street_id, std::string_view number) const {
  const Vrst_stmt s = prepare(*m_db, "SELECT lat, lon FROM addr WHERE street_id = ?1 AND number = ?2");
  sqlite3_bind_int64(s.get(), 1, street_id);
  bind_text(*s, 2, number);
  if (sqlite3_step(s.get()) != SQLITE_ROW) return std::nullopt;
  return Vrst_coord{sqlite3_column_int(s.get(), 0), sqlite3_column_int(s.get(), 1)};
}

}  // namespace

std::unique_ptr<Vrst_search_index_writer> make_sqlite_search_writer(const fs::path& dir) {
  return std::make_unique<Vrst_sqlite_writer>(dir);
}

std::unique_ptr<Vrst_search_index> open_sqlite_search_index(const fs::path& file) {
  return std::make_unique<Vrst_sqlite_index>(file);
}

}  // namespace nav
