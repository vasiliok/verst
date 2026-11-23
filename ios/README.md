# verst for iPhone

A thin UIKit app around the shared C++ core. It draws the same map as the desktop
viewer with the same code (`src/provider/map_scene.h`, `src/render/map_renderer.h`), using OpenGL ES 3.0
through a `GLKView`. Apple has deprecated OpenGL ES, but it still works on iOS 18 and
in the Simulator.

No map is bundled. The app downloads Geofabrik extracts, merges them when there are several,
and converts them on the phone with the same library as `osm2nav`. The phone stores several regions and
shows one at a time.

## Build and run in the Simulator

```sh
cmake -G Xcode -S ~/src/verst -B ~/src/tmp/verst/build-ios \
  -DCMAKE_SYSTEM_NAME=iOS -DCMAKE_OSX_SYSROOT=iphonesimulator \
  -DCMAKE_OSX_ARCHITECTURES=arm64 -DCMAKE_OSX_DEPLOYMENT_TARGET=15.0
cmake --build ~/src/tmp/verst/build-ios --config Debug -- -sdk iphonesimulator

xcrun simctl boot "iPhone 16 Pro"; open -a Simulator
xcrun simctl install booted ~/src/tmp/verst/build-ios/ios/Debug-iphonesimulator/Verst.app
xcrun simctl launch booted com.verst.app --download andorra
```

Use `-DCMAKE_OSX_ARCHITECTURES=x86_64` on an Intel Mac. A fresh install has no map: the
screen shows only the grid and the Maps screen opens by itself.

Launch arguments for development:

| Argument | Effect |
|---|---|
| `--download <id>[,<id>...]` | queue those Geofabrik extracts (ids from the catalog, e.g. `andorra`) as one region |
| `--import-region <folder>` | copy a folder made by `osm2nav` into the phone's regions (repeatable) |
| `--center <lat> <lon>` `--mpp <m>` | start view |
| `--gpx <file>` `--speed <x>` | replay a GPS track |
| `--follow follow\|heading` | start following the position |
| `--record <s>` / `--scenario <file>` | record for s seconds / play a scenario |
| `--console` | open the console at start |
| `--search <text>` / `--search-pick <text>` | open the search screen with that text / show its first result with the pin |
| `--share <text>` | as if the text (coordinates or a map link) was shared from Google Maps: pin and route to it |
| `--log-dir <dir>` | where the session log goes |

## Run on an iPhone

Signing needs an Apple ID added in Xcode (Settings > Accounts). Configure with
`-DCMAKE_OSX_SYSROOT=iphoneos -DCMAKE_OSX_ARCHITECTURES=arm64 -DVERST_IOS_TEAM=<team id>
-DVERST_BUNDLE_ID=<your bundle id>`, then build and install:

```sh
xcodebuild -project verst.xcodeproj -scheme verst_ios -configuration Debug \
  -destination 'generic/platform=iOS' -allowProvisioningUpdates build
xcrun devicectl device install app --device <device id> ios/Debug-iphoneos/Verst.app
```

`xcrun devicectl list devices` shows the device id. Wi-Fi works when the phone and the
Mac are on the same network. The first launch needs Settings > General > VPN & Device
Management > Trust. Apps signed with a free team expire after 7 days.

## Maps screen

`Maps` in the toolbar (it shows the progress, e.g. "Maps 45%", while a job runs) opens a list
with three sections:

- **In progress**: one row per job with its state. A job is Queued, Downloading (percent,
  MB, speed, time left), Paused, Merging, Converting (phase and percent), Cancelled or
  Failed (with the reason, e.g. not enough memory). Tap a row to Pause / Resume / Cancel,
  or Retry / Remove. One download and one conversion run at a time; jobs survive an app restart,
  and a paused download continues from the bytes it kept.
- **On this phone**: the ready regions with size and date. The shown one has a checkmark.
  Tap one to switch to it (the camera of each region is remembered). Swipe left to
  delete one, except the shown region.
- **Download more**: the Geofabrik catalog as a tree (continent > country > region).
  Pick one extract, or several neighbours to join into one region, then name it.

Regions live in Documents/regions/<id>/: the nav files, the `.poly` of each extract and
`region.json`. The region border is drawn, and outside it the map is dimmed and gets a grid
whose spacing follows the zoom. A region without a `.poly` still opens, with no border.

## Using it

| Gesture / button | Action |
|---|---|
| one-finger drag | pan |
| pinch | zoom around the fingers (and pan with them) |
| two-finger twist | rotate |
| double tap / two-finger tap | zoom in / out 2x |
| Fit / Start | whole map / start view |
| Roads | menu with the 9 road class groups, plus Highlight oneway and Show vertices |
| Maps | the Maps screen (above) |
| Names | show place names |
| Log | full-screen console (see below) |
| red record button (round, right) | start recording what you do with the map; tap again (stop square) to stop and save |
| green play button (above it) | play a GPS track (a GPX in the app's tracks folder, or "Open GPX file..." from Files, iCloud Drive or AirDrop: copied in and played) or a saved recording (Play or Share...). While a track is loaded: Pause / Resume, speed x1 / x5 / x10 / x30, Follow track, Other track..., Close track. The replayed track is the GPS position meanwhile, so follow and heading-up behave as when driving. While a recording plays, tap (stop square) to stop |
| hold the map 1 s | drops a pin; its card names the nearest road and offers Car / Bike / Walk (remembered), **Start here** (this pin starts the next route, green) and **Route** (from the start pin, or the GPS position; a replayed track counts). The route is drawn in blue, fitted on screen, with its length and time. The way back is computed with it (on the same roads, detouring only where a oneway forbids them), kept and saved per region but not drawn: **Way back** on the card shows it (**There** switches back), also after the app was closed. x clears pins, route and the saved way back. A point shared from Google Maps is routed to at once |
| magnifier (round, under the eye) | address search: results as you type (house number, street, place, nearest first); a tap moves the map there and puts a red pin; "Clear pin" in the same screen removes it |
| eye (round, top right) | hide or show the black info boxes (stats box and debug overlay) for a clean map; logging goes on |
| + / - (round, bottom left) | zoom in / out 2x around the screen center, animated; taps add up |
| location arrow (round, bottom right) | tap: follow your GPS position (map keeps you centred); tap again: heading up (map turns with your direction of travel, you sit lower on screen to see ahead); tap again: follow, north up. A drag stops following; a two-finger twist leaves heading up. Stops following a replayed track |
| tap on the stats box | debug menu: overlay, send logs, tests, tracks |

The stats box shows V, S, A, P, the zoom relative to the whole map, metres per
screen point and the map provider's work: "loading done", or while pieces load the
missing / queued / working / ready counts, busy workers of all threads, totals
(built, failed, dropped) and what each worker builds and for how long.

**Console** (`Log`, or `--console` at launch): splits the screen, the console on top
(45% of the height) and the map below it, still usable, so you pan and zoom while you
watch the map provider work. `Full` gives the console the whole screen, `Split` goes
back, `Close` (or `Log` again) gives the map the whole screen. In Full the top shows, live, the map
description and the provider status above (in Split the map's stats box shows them). Below: the session log as it is written (the
last 2000 records), newest at the bottom; it follows new lines while scrolled to the end
and holds still while you read older ones. Filter buttons: `Map` (the provider's work:
build, queue, load, map, error), `User` (touch, gesture, ui), `Events` (all but the
per-frame frame, tile, touch and queue records) and `All`; the filter field keeps only
lines containing its text (e.g. `failed`, `stall`, `10/523`).

**Recording.** The recording follows the camera, whatever moves it: gestures, Fit, Start,
the location button. It is saved as a scenario `rec-<date>-<time>.txt` in
Documents/scenarios (visible in the Files app, so it can be shared or deleted there):
`view <s> <lat> <lon> <m/pt> <deg>` lines (camera, at most 30 per second) and `wait`
lines for pauses. Playing moves the camera the same way with the same timing, on any
phone; a touch stops it. Toggles (Names, Oneway, Dots, Roads) are not recorded.
`--record <s>` at launch records for s seconds from the first frame.
