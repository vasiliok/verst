# session

[verst](../../README.md) > [src](../README.md) > session

What happens during an app run: logs, scripted test motion, recorded user motion, GPS
tracks. Platform independent; the desktop viewer and the iOS app use the same code.

| File | What |
|---|---|
| `session_log.{h,cpp}` | `Vrst_session_log`: one JSON Lines file per run (frame timing, touches, gestures, tile builds, stalls, routes) |
| `scenario.{h,cpp}` | `Vrst_scenario`: scripted map motion (scroll, fling, zoom, rotate, jump) so a performance run is the same every time |
| `activity_recorder.{h,cpp}` | `Vrst_activity_recorder`: records what the user does with the map as a scenario that plays back the same way |
| `gpx_track.{h,cpp}` | `Vrst_gpx_track`: a GPX track with time-based playback queries (GPX replay acts as the GPS position in the app) |

Uses: geometry, provider (`map_scene.h`).
Used by: navview, the iOS app.
