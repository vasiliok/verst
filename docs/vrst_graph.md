# How Vrst_graph works

`Vrst_graph` (in `src/nav_files.h`) is a read-only view of `graph.dat`. It owns no
data of its own: every member is a `std::span` into the memory-mapped file, so opening
a graph parses and copies nothing.

## 1. What it stores

V is the number of vertices and A the number of directed arcs.

| Member            | Type and size                | Meaning                                                    |
|-------------------|------------------------------|------------------------------------------------------------|
| `m_file`          | `Vrst_nav_file`              | Owns the mmap of `graph.dat`; every span points into it.   |
| `m_vertex_coord`  | `Vrst_coord[V]`, 8 B each    | Position of vertex `v`, degrees x 10^7.                     |
| `m_fwd_first_out` | `uint32[V+1]`                | Where each vertex's outgoing arcs start in `m_fwd_arcs`.   |
| `m_fwd_arcs`      | `Vrst_fwd_arc[A]`, 12 B each | `{head, weight, seg}`, grouped by tail vertex.             |
| `m_bwd_first_in`  | `uint32[V+1]`                | Where each vertex's incoming arcs start in `m_bwd_arcs`.   |
| `m_bwd_arcs`      | `Vrst_bwd_arc[A]`, 12 B each | `{tail, weight, fwd_arc}`, grouped by head vertex.         |
| `m_bbox`          | `Vrst_bbox`, copied          | Extent of all source nodes.                                |

- A **vertex** is an OSM node where a road ends or where two or more road ways meet.
- A **segment** is the road piece between two vertices, stored once in `edges.dat`.
- An **arc** is one direction a car may drive along a segment: a two-way road has
  2 arcs, a oneway road has 1.

## 2. Forward CSR

All arcs sit in one array, sorted by tail vertex, and each vertex owns a contiguous
slice of it:

```
out-arcs of v = fwd_arcs[ fwd_first_out[v] .. fwd_first_out[v+1] )
```

The tail of an arc is not stored; it is implied by the slice the arc is in, which
saves 4 bytes per arc. `fwd_first_out` has V+1 entries and the last one equals A, so
the last vertex needs no special case.

Example: three vertices, 0 -(two-way, segment 7)- 1 -(oneway 1->2, segment 9)- 2.

```
fwd_first_out = [0, 1, 3, 3]          V+1 = 4 entries, last = A = 3
fwd_arcs      = [ {head 1, w, seg 7},              <- vertex 0: [0,1)
                  {head 0, w, seg 7|bit31},        <- vertex 1: [1,3)
                  {head 2, w, seg 9} ]
                                                    vertex 2: [3,3) no out-arcs
```

`out_arcs(v)` returns `fwd_arcs.subspan(first_out[v], first_out[v+1] - first_out[v])`:
two array reads and no search, so it is O(1).

## 3. The fields of an arc

- **`head`**: the target vertex.
- **`weight`**: travel time in deciseconds, computed by osm2nav as
  `max(1, round(length_dm x 3.6 / speed_kmh))`.
  - The speed is the tagged maxspeed, or the road class default when there is none.
  - Deciseconds keep short city segments from all rounding to the same time.
  - The minimum of 1 ensures no arc costs 0.
- **`seg`**: a segment id plus a direction bit.
  - `seg & kSegIdMask` indexes `edges.dat` (attributes, name, OSM way) and
    `geom.dat` (shape).
  - Bit 31 (`kSegReversed`) set means the arc runs from the segment's `b` end to its
    `a` end, so its shape must be read backwards. In the example the 1->0 arc carries
    `7|bit31`.
  - A `oneway=-1` road has only the reversed arc.

## 4. Backward CSR

`bwd_first_in` and `bwd_arcs` hold the same A arcs again, grouped by **head**:

```
in-arcs of v = bwd_arcs[ bwd_first_in[v] .. bwd_first_in[v+1] )
Vrst_bwd_arc = {tail, weight, fwd_arc}
```

- **Why it exists:** bidirectional Dijkstra and contraction hierarchies also search
  backward from the target. Without this index, finding the arcs that enter v would
  mean scanning all A arcs.
- **`weight`** is copied from the forward arc, so the backward search stays inside
  this array and avoids random jumps into `fwd_arcs`.
- **`fwd_arc`** is the index of the same arc in `fwd_arcs`. When the two searches
  meet, it turns the backward half of the path into forward arcs, which carry the
  segment and shape.

In the example, vertex 2 has one incoming arc: `{tail 1, w, fwd_arc 2}`.

## 5. How osm2nav builds it

1. Each segment produces its arcs, in segment order, as temporary records with an
   explicit tail:
   - `Dir::Both` gives a->b and b->a;
   - `Dir::Fwd` gives only a->b;
   - `Dir::Bwd` gives only b->a, with bit 31 set.
2. **Forward CSR**: a counting sort by tail.
   - Count the arcs per tail into `fwd_first_out[tail+1]`.
   - Take the prefix sum.
   - Place each arc at `pos[tail]++`.
   - The sort is stable, so within one vertex the arcs stay in segment order.
3. **Backward CSR**: the same counting sort, by head, run over `fwd_arcs` in index
   order. As a result, the in-arcs of a vertex are ordered by tail.
4. Vertex ids come from `number_vertices()`, currently in OSM node id order. A TODO
   there plans to sort them by Hilbert curve key, so that vertices close on the map
   are close in memory.

## 6. What open() does

1. `m_file.open(path, FileType::Graph)` maps the file and checks:
   - the magic `NAVGRAPH`;
   - `format_version`;
   - `file_type`;
   - that every section is 64-byte aligned and inside the file.
2. It fetches the six sections by id with `section<T>()`, which refuses any section
   whose stored `elem_size` differs from `sizeof(T)`.
3. It checks that the sizes agree:
   - `fwd_first_out` and `bwd_first_in` both have V+1 entries;
   - `bwd_arcs` has the same count as `fwd_arcs`;
   - the bbox section has exactly 1 element.
4. It copies the bbox. Everything else stays in the mapped file.

`open()` does not check the contents (monotonic offsets, heads below V, the
backward/forward one-to-one match and so on). Those deeper checks are in navcheck.

## 7. File layout and cost

```
graph.dat: header 64 B | 6 section entries x 24 B | (pad to 64) |
           VertexCoord 8V | FwdFirstOut 4(V+1) | FwdArcs 12A |
           BwdFirstIn 4(V+1) | BwdArcs 12A | Bbox 16   (each section 64-B aligned)
```

Andorra has V = 5,019 and A = 9,657, which comes to about 312 KB in total.

| Operation                   | Cost                                                         |
|-----------------------------|--------------------------------------------------------------|
| `open()`                    | about O(1): map the file and walk the 6 section entries      |
| `out_arcs(v)`, `in_arcs(v)` | O(1)                                                         |
| tail of a forward arc       | not stored; binary search in `fwd_first_out` (navcheck's `tail_of`) or read it from the matching backward arc |
| segment of an arc           | `seg & kSegIdMask`, O(1)                                     |

## 8. What is deliberately not in it

Names, road class, length, flags and shape points are not in the graph; they live in
`edges.dat`, `geom.dat` and `names.dat`. That way a route search only touches the
pages of `graph.dat`. After the search, each arc's `seg` leads to the other files for
guidance and drawing.

## 9. How it fits with the other classes in nav_files.h

`nav_files.h` connects its classes in two ways: ownership (which object holds which,
from the mapped file up to the full data set) and ids (how an index stored in one
file points into another file).

### Ownership

```
Vrst_nav_data                       one import = 4 files
 +-- m_graph : Vrst_graph           graph.dat
 |     +-- m_file : Vrst_nav_file
 |           +-- m_file : Vrst_mapped_file   (mmap)
 +-- m_edges : Vrst_edges           edges.dat   -+
 +-- m_geom  : Vrst_geom            geom.dat     + same shape: Vrst_nav_file + typed spans
 +-- m_names : Vrst_names           names.dat   -+
```

Each layer has one job and one set of checks:

| Class | Job | What it checks |
|---|---|---|
| `Vrst_mapped_file` (in `mapped_file.h`) | Maps a file read-only. Move-only, so only one owner ever unmaps it. | That the file can be opened, stat'ed and mapped. |
| `Vrst_nav_file` | Turns the mapped bytes into typed sections: `section<T>(id)` returns a `std::span<const T>`. | Magic, `format_version` and `file_type`; every section is 64-byte aligned and inside the file; `elem_size == sizeof(T)`. |
| `Vrst_graph`, `Vrst_edges`, `Vrst_geom`, `Vrst_names` | Typed view of one file. Each fetches its sections in `open()` and offers getters plus small helpers (`out_arcs`, `segment_points`, `get`). | Sizes agree within the file, e.g. `fwd_first_out` has V+1 entries and all per-segment arrays have S entries. |
| `Vrst_nav_data` | The four files together; the normal entry point for tools. | Checks that span files: all four share one `build_id`, and `geom` has S+1 offsets, matching `edges`. |

- **Lifetime:** every span points into the memory of the `Vrst_mapped_file` at the
  bottom. A span is valid only while the `Vrst_nav_data` (or the single view) that
  owns it is alive. Nothing is copied except the one-element bbox.
- **Opening:** `Vrst_nav_data::open(dir)` calls `open_file()` for each of the four
  files, then runs the cross-file checks. navcheck calls `open_file()` itself, one
  file at a time, so it can report every broken file and not just the first.

### How ids link the files

The files contain no pointers; integer ids are the only links between them:

```
          graph.dat                         edges.dat                    names.dat
 vertex v -- vertex_coord[v]
          -- out_arcs(v) -+
                          | Vrst_fwd_arc.seg & kSegIdMask = s
                          +--------------> attr[s] ---- name_id, ref_id --> get(id)
                                           vertices[s] = {a, b} --> back to vertices
                                           osm_way[s]  (OSM way id)
                                              | same s
                                              v
                                          geom.dat
                                           segment_points(s) = points[first[s] .. first[s+1])
```

| From | Id | To |
|---|---|---|
| `Vrst_fwd_arc.head` | vertex id | `vertex_coord`, `out_arcs`, `in_arcs` in the graph |
| `Vrst_bwd_arc.tail` | vertex id | same as above |
| `Vrst_bwd_arc.fwd_arc` | arc index | `fwd_arcs[i]`, the same arc seen forward |
| `Vrst_fwd_arc.seg & kSegIdMask` | segment id s | `attr[s]`, `vertices[s]`, `osm_way[s]` in edges; `segment_points(s)` in geom |
| `Vrst_fwd_arc.seg` bit 31 | direction | the arc runs b->a: read `segment_points(s)` backwards |
| `Vrst_seg_vertices {a, b}` | vertex ids | back into the graph; a is the first shape point, b the last |
| `Vrst_seg_attr.name_id`, `ref_id` | string id | `names.get(id)`; 0 means "" |

**Why build_id matters:** every id above is only valid within one import. A
`graph.dat` from one run combined with an `edges.dat` from another can have identical
sizes and still be wrong everywhere. `Vrst_nav_data::open()` refuses such a mix.

### Example: from an arc to what is shown or spoken

```cpp
const Vrst_nav_data& d = ...;
for (const Vrst_fwd_arc& a : d.graph().out_arcs(v)) {
  const Vrst_uint32 s = a.seg & kSegIdMask;
  const Vrst_seg_attr& at = d.edges().attr()[s];            // class, length, flags
  std::string_view name = d.names().get(at.name_id);        // "Avinguda Meritxell"
  auto pts = d.geom().segment_points(s);                    // shape, a -> b
  bool reversed = a.seg & kSegReversed;                     // draw/guide pts backwards
  Vrst_coord target = d.graph().vertex_coord()[a.head];
}
```

The router only ever reads `d.graph()`. The edges, geom and names files are read only
after a route is found, for guidance and drawing. That is why they are separate files
and separate classes: a route search never loads their pages.
