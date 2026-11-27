# verst development pipeline: fix -> check -> Simulator -> iPhone -> UI test scenarios

[verst](../README.md) > [docs](../README.md#documentation-map) > dev-pipeline

State of 2026-11-25.

## 0. Rules (user)

- Code in `~/src/verst`; every build, log, screenshot and script in `~/src/tmp/verst`.
- Tools may only touch `~/src`, `~/.claude`, `~/Documents` (global hook). The hook also
  blocks Bash commands containing `//`, `///` or `/word` patterns (sed or grep on Doxygen
  comments, `/usr/libexec/...`): use a python script in `~/src/tmp/verst`, or Read/Edit.
- No desktop checks as tests: no ctest, no unit-test runs, no navview runs as proof.
  Unit tests may be written, not run.
- Simulator = only "does the new feature work" (it runs, moves, logs). Never performance.
- Performance and feel = only the real iPhone 13, with session logs and screen recordings.
- Slow or laggy report: logs and recordings first, then only the code they point at.
- Never commit or push without being asked. Commit message = exactly 3 lines, no blank
  line, no trailers.
- Code rules: Vrst_ names, Doxygen on everything, ASCII only, no raw pointers, .h + .cpp
  split, includes as `"dir/file.h"` relative to `src/`.

## 1. Fix

1. Read the log or recording the user sent (section 6) before the code.
2. Edit in `~/src/verst`. New sources: add them to `CMakeLists.txt` (vrst_core,
   vrst_render, vrst_convert) or `ios/CMakeLists.txt`, then **re-run cmake for each build
   dir**: an old Xcode project will not see new files (link errors "missing classes").
3. Desktop compile check of the core and tools (compiler errors and warnings only, not
   a test). Build dir: `~/src/tmp/verst/build-readme` (Release, Makefiles):

```sh
cd ~/src/tmp/verst
cmake -S ~/src/verst -B build-readme > cfg.log 2>&1
cmake --build build-readme -j8 > build.log 2>&1; echo $?; grep -E "warning|error" build.log
```

4. Data-side checks with the dev tools on a converted map (Bourgogne, format 4:
   `~/src/tmp/verst/readme-check/out/bourgogne`; bigger BFC map: `~/src/tmp/verst/nav-bfc`):

```sh
build-readme/navroute readme-check/out/bourgogne 47.3220 5.0415 47.0260 4.8400 car   # Dijon -> Beaune
#   expect: car 45.69 km, 28 min; back 46.72 km, 91% on the outbound roads
build-readme/navroute readme-check/out/bourgogne 47.3220 5.0415 47.3300 5.0300 foot  # back 100%
build-readme/navsearch readme-check/out/bourgogne "12 rue de la liberte dijon"
build-readme/navcheck  readme-check/out/bourgogne                                      # exit 0 = PASS
build-readme/osm2nav <in.osm.pbf> <out_dir>                                            # prints phase times + peak RSS
```

## 2. Simulator (functional check)

- iPhone 16 Simulator `<SIM_ID>` (`xcrun simctl list devices`; ours in `notes/devices.md`). The Mac is Intel: always
  `x86_64` (an arm64 build does not install).
- Build dir `~/src/tmp/verst/build-ios-sim` (Xcode generator, sysroot iphonesimulator).
  Scheme is **verst_ios**. Bundle id in this build: **com.verst.app** (check with
  `plutil -extract CFBundleIdentifier raw <app>/Info.plist`).
- `open -a Simulator` first so the user sees the window (`simctl boot` alone is headless).

```sh
cd ~/src/tmp/verst
cmake -S ~/src/verst -B build-ios-sim > sim-cfg.log 2>&1
xcodebuild -project build-ios-sim/verst.xcodeproj -scheme verst_ios -configuration Debug \
  -sdk iphonesimulator -arch x86_64 build > sim.log 2>&1; echo $?
grep "warning:" sim.log | grep "src/verst" | sort -u     # our warnings only (freetype ones are third-party)
open -a Simulator; xcrun simctl boot <SIM_ID>
xcrun simctl install <SIM_ID> build-ios-sim/ios/Debug-iphonesimulator/Verst.app
xcrun simctl launch --terminate-running-process <SIM_ID> com.verst.app <args>
sleep 8; xcrun simctl io <SIM_ID> screenshot shot.png   # then Read the png
```

App launch arguments (ios/main.mm), handy to drive a feature without hands:

| Argument | What |
|---|---|
| `--center <lat> <lon>` `--mpp <m>` | start camera |
| `--follow` | GPS follow mode |
| `--gpx <file>` `--speed <x>` | GPX replay as the GPS position |
| `--search <text>` `--search-pick <n>` | run a search, pick result n (pin) |
| `--share <text>` | as if shared from Google Maps (route mode to that point) |
| `--scenario <name or Mac file>` | play a UI test scenario (section 5) |
| `--record` | record the camera path as a scenario |
| `--log-dir <dir>` | write the session log there (Simulator: a Mac path) |
| `--console` | open the debug console |
| `--download` `--import-region <dir>` | pipeline / region import tests |
| `--stuck-ms <ms>` | main-thread stall threshold for the log |

## 3. Real iPhone (install + measure)

- Phone ids (`<PHONE_UDID>`, `<DEVICECTL_ID>`) and the signing team are in
  `notes/devices.md` (not in git).
- Free signing team, bundle `com.albacil.verst`, share extension
  `com.albacil.verst.share`. Free signing expires after 7 days: reinstall.
- Build dir `~/src/tmp/verst/build-ios-device` (iphoneos, arm64, VERST_IOS_TEAM,
  VERST_BUNDLE_ID). Must build with the scheme AND the phone as destination, or the
  profile lacks the phone.

```sh
cd ~/src/tmp/verst
cmake -S ~/src/verst -B build-ios-device > dev-cfg.log 2>&1
xcodebuild -project build-ios-device/verst.xcodeproj -scheme verst_ios -configuration Debug \
  -destination 'id=<PHONE_UDID>' -allowProvisioningUpdates \
  -allowProvisioningDeviceRegistration build > dev.log 2>&1; echo $?; grep -c "BUILD SUCCEEDED" dev.log
xcrun devicectl device install app --device <DEVICECTL_ID> \
  build-ios-device/ios/Debug-iphoneos/Verst.app
```

- devicectl over Wi-Fi sometimes resets: just retry. A locked phone refuses launches.
- First install on a new phone: Settings > General > VPN & Device Management > Trust.
- Then ask the user to try it and screen-record (fps, frame time and tiles are on screen
  in the debug boxes; the eye button hides them).

## 4. Logs

- Every run writes a session log: JSON Lines, `Documents/logs/session-<date>.jsonl`
  on the phone (records: launch, frame, touch, gesture, build, stall, route, search,
  share, scenario, ui, system...).
- From the phone by cable:
  - `sh ~/src/tmp/verst/tools/run_scenario.sh <scenario> <seconds>` - launches a scenario on
    the iPhone 13 and copies its log into `logs/iphone13/`.
  - `sh ~/src/tmp/verst/tools/follow_log.sh [every s] [max min]` - copies the newest log
    every few seconds while the user plays (`logs/iphone13/live`).
  - Manual: `xcrun devicectl device copy from --device <id> --domain-type appDataContainer
    --domain-identifier com.albacil.verst --source Documents/logs/<name> --destination <file>`.
- From the phone without cable: debug menu "Send this session" (share sheet / AirDrop).
  Never read `~/Downloads` yourself: ask the user to run
  `! cp ~/Downloads/rec-*.txt ~/Downloads/session-*.jsonl ~/src/tmp/verst/recordings/`.
- Analyse: `python3 ~/src/tmp/verst/tools/analyze_log.py <session.jsonl> [...] [--csv out.csv]`
  (validation, launch, per-gesture frames and lag, per-scenario-step fps, slow frames > 33 ms
  and their phase, stalls, tile builds by zoom).

## 5. Saved UI tests (scenarios)

A scenario is a text file of map motion the app plays by itself, so a run is the same
every time and needs no hands (format in `src/session/scenario.h`):

```
# comment
goto <lat> <lon> <m/pt>          jump
wait <s>                         pause
wait_tiles <max s>               until nothing is missing
pan <dx> <dy> <s>                scroll (screen points)
fling <dx> <dy> <s>              fast then slower
zoom <factor> <s>                pinch; zoom_in / zoom_out = 2x / 0.5x at once
rotate <deg> <s>                 counter-clockwise
view <s> <lat> <lon> <m/pt> <deg>  move to a camera (what the recorder writes)
```

Motion follows the clock, not frames: a slow phone shows fewer frames of the same motion.

- **Bundled scenarios** (`ios/scenarios/*.txt`, copied into the app, listed in the debug
  menu): `cities`, `idle`, `pan-dijon`, `taps`, `travel`, `zoom-fast`, `zoom-ladder`.
  Add one = new .txt in `ios/scenarios/` + rebuild.
- **Run on the iPhone:** debug menu (play list), or
  `sh ~/src/tmp/verst/tools/run_scenario.sh cities 60` (launch with `--scenario`, wait,
  fetch the log). A touch stops a running scenario ("TEST stopped by touch").
- **Run in the Simulator:** `sh ~/src/tmp/verst/tools/sim_scenario.sh <name or Mac file> <out dir> <timeout s>`
  (waits for `"event":"finished"`, screenshot `end.png`). `tools/sim_run.py` does the same
  with arbitrary app args and prints chosen record types. NOTE: both scripts still say
  `APP=com.verst.bourgogne` (old build dir build-ios-bourgogne); for build-ios-sim use
  `com.verst.app`.
- **Record a user session as a test:** in the app, red round button = record the camera
  path -> `rec-<date>.txt` (`view`/`wait` steps); green button plays it; play list ->
  Share... sends it. Typical compare loop:
  1. user records on the iPhone, sends `rec-*.txt` + the session log, cp into
     `~/src/tmp/verst/recordings/`;
  2. replay the same file in the Simulator (`sim_scenario.sh recordings/rec-....txt logs/<dir> <timeout>`)
     or on the phone;
  3. `python3 tools/compare_runs.py <phone.jsonl> <replay.jsonl>` compares the recorded part
     with its replay; `tools/record_roundtrip.py` checks record -> replay ends at the same camera.
- **From a video:** `tools/make_laignes_scenario.py` turned a 59 s screen recording into
  `ios/scenarios/laignes-video.txt` (approximate pans and pinches) - the pattern for
  re-creating a user's video as a test.
- **GPX as GPS:** `--gpx <file> --speed <x>` (or the play button in the app) replays a
  drive, e.g. to check follow mode and routing while "moving".

## 6. Bug / performance loop (summary)

user report -> ask for recording + session log (or fetch by cable) -> `analyze_log.py`
-> findings with numbers -> code only where the log points -> desktop compile ->
Simulator: does it work (scenario / launch args, screenshot) -> iPhone build + install ->
same scenario on the phone (`run_scenario.sh`) or user recording -> compare logs before /
after -> user says commit -> 3-line commit -> push when asked.
