# verst for iPhone {#ios_readme}

A thin UIKit app around the shared C++ core. It draws the same map as the desktop
viewer with the same code (`src/map_scene.h`, `src/map_renderer.h`), using OpenGL ES 3.0
through a `GLKView`. Apple has deprecated OpenGL ES, but it still works on iOS 18 and
in the Simulator.

## Build and run in the Simulator

The app bundles one converted map. Convert an extract with osm2nav first; the desktop
tests already leave Andorra in `<build>/test_out`.

```sh
cmake -G Xcode -S ~/src/verst -B ~/src/tmp/verst/build-ios \
  -DCMAKE_SYSTEM_NAME=iOS -DCMAKE_OSX_SYSROOT=iphonesimulator \
  -DCMAKE_OSX_ARCHITECTURES=x86_64 -DCMAKE_OSX_DEPLOYMENT_TARGET=15.0 \
  -DVERST_MAP_DIR=$HOME/src/tmp/verst/build/test_out
cmake --build ~/src/tmp/verst/build-ios --config Debug -- -sdk iphonesimulator

xcrun simctl boot "iPhone 16 Pro"; open -a Simulator
xcrun simctl install booted ~/src/tmp/verst/build-ios/ios/Debug-iphonesimulator/Verst.app
xcrun simctl launch booted com.verst.app --center 42.48014 1.45493 --mpp 1.1
```

Use `-DCMAKE_OSX_ARCHITECTURES=arm64` on an Apple silicon Mac. Without launch
arguments the app starts at the default point, or the whole map if that point is
outside the data.

## Run on an iPhone

Signing needs an Apple ID added in Xcode (Settings > Accounts). Then configure with
`-DCMAKE_OSX_SYSROOT=iphoneos -DCMAKE_OSX_ARCHITECTURES=arm64 -DVERST_IOS_TEAM=<team id>`
and run the `verst_ios` target from the generated `verst.xcodeproj` in Xcode.

## Using it

| Gesture / button | Action |
|---|---|
| one-finger drag | pan |
| pinch | zoom around the fingers (and pan with them) |
| two-finger twist | rotate |
| double tap / two-finger tap | zoom in / out 2x |
| Fit / Start | whole map / start view |
| Roads | menu with the 9 road class groups |
| Oneway / Dots / Names | highlight oneway roads / show graph vertices / show place names |

The stats line shows V, S, A, P, the zoom relative to the whole map, metres per
screen point and the dead-end count.
