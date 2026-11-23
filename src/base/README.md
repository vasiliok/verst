# base

Small building blocks used by every other part. No dependencies inside `src/`.

| File | What |
|---|---|
| `vrst_types.h.in` | project-wide scalar aliases (`Vrst_int32`, `Vrst_float64`, ...); CMake generates `<build>/generated/vrst_types.h` from it (`VRST_TYPE_*` cache variables) |
| `mapped_file.{h,cpp}` | `Vrst_mapped_file`: read-only memory mapping of a whole file (POSIX and Windows) |
| `string_pool.h` | `Vrst_string_pool`: strings interned to dense ids (names.dat, address index) |
| `convert_progress.h` | live progress and cancel switch of the map pipeline (merge, convert), shared by the tools and the app |

Used by: storage (mapped files), convert (string pool, progress), and the iOS download
manager (progress).
