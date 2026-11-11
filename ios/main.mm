/// @file
/// verst iPhone app: a full-screen map drawn with OpenGL ES 3.0.
///
/// A thin UIKit shell around the shared C++ core: open_map() gives the map provider
/// for the bundled map, which delivers ready-to-draw pieces, and Vrst_tile_renderer
/// draws them, exactly as in the desktop viewer. The app does not know how the map
/// is stored. All map layers
/// are always on. This file only adds the GL context (GLKView), touch gestures, the
/// GPS position (CoreLocation), a stats line and a toolbar.
///
/// Gestures: one-finger drag = pan, pinch = zoom (and pan with the two fingers),
/// two-finger twist = rotate, double tap = zoom in, two-finger tap = zoom out.
/// Launch arguments `--center <lat> <lon>` and `--mpp <metres>` set the start view,
/// like navview; `--log-dir <dir>` puts the session logs elsewhere (Simulator);
/// `--scenario <name>` plays a test scenario (scenario.h) after the first frame;
/// `--stuck-ms <ms>` logs a worker as "stuck" sooner than kStuckMs (to test the dump);
/// `--record <s>` records the camera path for s seconds from the first frame (debug
/// menu "Record activity" does the same by hand; the result plays as a scenario);
/// `--gpx <file>` replays a GPX track (Documents/tracks or a full path) at `--speed <x>`
/// times its recorded pace (default 10), the camera following the marker. VrstDebugOverlay (clock, frame numbers, touches and gesture names)
/// is on at start for screen recordings; a tap on the stats box hides or shows it.
/// The "Log" button (or `--console` at launch) splits the screen: VrstDebugConsole on
/// top (the map provider's status live and the session log as it is written), the map
/// below, still usable; the console can also take the whole screen.

#include "gl_api.h"  // OpenGL ES 3.0; before GLKit so the ES3 header wins

#import <CoreLocation/CoreLocation.h>
#import <GLKit/GLKit.h>
#import <UIKit/UIKit.h>

#import "debug_console.h"
#import "debug_overlay.h"
#import "root_controller.h"

#include <sys/utsname.h>

#include <algorithm>
#include <cmath>
#include <exception>
#include <memory>
#include <optional>
#include <set>
#include <stdexcept>

#include "glyph_atlas.h"
#include "gpx_track.h"
#include "map_labels.h"
#include "map_renderer.h"
#include "map_provider.h"
#include "map_scene.h"
#include "activity_recorder.h"
#include "scenario.h"
#include "session_log.h"
#include "tile_renderer.h"

using namespace nav;

/// GPU byte budget of the tile cache; a phone has less memory to spare than a desktop.
static constexpr Vrst_size kTileBudget = 96u << 20;
/// Session log files kept on the phone; older ones are deleted at launch.
static constexpr Vrst_size kKeepSessions = 20;
/// A display-link gap longer than this (ms) is logged as a main-thread stall.
static constexpr double kStallMs = 34;
/// Side of the round buttons on the map (location, record, play), points.
static constexpr CGFloat kRoundButtonSize = 44;
/// Build configuration, for the launch record (Debug code runs several times slower).
#ifdef NDEBUG
static constexpr const char* kBuildConfig = "release";
#else
static constexpr const char* kBuildConfig = "debug";
#endif
/// CPU architecture, for the launch record (floating point may round differently).
#if defined(__aarch64__)
static constexpr const char* kArch = "arm64";
#elif defined(__x86_64__)
static constexpr const char* kArch = "x86_64";
#else
static constexpr const char* kArch = "other";
#endif
/// A worker this long on one piece is logged as "stuck" with its exact input, ms.
static constexpr double kStuckMs = 2000;
/// Most "stuck" records per session (each may hold thousands of points).
static constexpr Vrst_size kMaxStuckDumps = 20;
/// File name start of recorded activity in Documents/scenarios.
static NSString* const kRecordingPrefix = @"rec-";
/// Seconds between two console refreshes while it is open.
static constexpr double kConsoleSeconds = 0.25;

/// Map controller (whole screen, or below the console in Split). Owns the shared C++ map state and turns UIKit touch
/// gestures into camera changes; draws only when something changed.
@interface VrstMapController : GLKViewController <UIGestureRecognizerDelegate, CLLocationManagerDelegate>
/// Debug console with the provider status and the session log ("Log" button); placed
/// by VrstRootController, fed by this controller. Set before the view loads.
@property(nonatomic, strong) VrstDebugConsole* console;
@end

@implementation VrstMapController {
  /// Map data of the app bundle: pieces, places, status; how it is stored is hidden.
  std::unique_ptr<Vrst_map_provider> _map;
  /// World origin, start views and the area on screen, from the map extent.
  Vrst_map_frame _mapFrame;
  /// Provider work after the last frame: threads, busy workers, settled.
  Vrst_provider_status _mapStatus;
  /// GPU tile cache and map drawing.
  Vrst_tile_renderer _tiles;
  /// Frame counter for the tile cache.
  Vrst_uint64 _frame;
  /// Overlay renderer: GPS position marker and labels.
  Vrst_map_renderer _renderer;
  /// Place name labels, laid out on every draw.
  Vrst_map_labels _labels;
  /// SDF glyphs of the label characters, from the bundled fonts.
  Vrst_glyph_atlas _atlas;
  /// Draw place names (Names button).
  BOOL _showLabels;
  /// Current camera.
  Vrst_view _view;
  /// Display switches changed by the toolbar; map layers stay at their default (all on).
  Vrst_tile_draw_options _opt;
  /// GPS position updates.
  CLLocationManager* _location;
  /// A GPS position is known.
  BOOL _hasFix;
  /// Last GPS position.
  Vrst_latlon _fix;
  /// Horizontal accuracy of _fix in metres.
  Vrst_float64 _fixAccuracy;
  /// Start view center: kDefaultPoint or --center.
  Vrst_latlon _startCenter;
  /// Start zoom in ground metres per screen point: kDefaultMetresPerPoint or --mpp.
  Vrst_float64 _startMpp;
  /// Scale that fits the whole map: zoom 1x in the stats and the zoom-out limit.
  Vrst_float64 _initialScale;
  /// Data opened and mesh built.
  BOOL _loaded;
  /// Renderer initialised.
  BOOL _glReady;
  /// Start view applied; needs the drawable size, known only at the first draw.
  BOOL _viewSet;
  /// Pinch centroid of the previous pinch event, in view points, for two-finger pan.
  CGPoint _lastCentroid;
  /// Touch count of the previous pinch event; a change means the centroid jumps.
  NSUInteger _lastTouches;
  /// OpenGL ES 3.0 context of the GLKView.
  EAGLContext* _context;
  /// Counts, zoom and errors in the top-left corner.
  UILabel* _stats;
  /// Toggle buttons, kept to update their tint.
  UIBarButtonItem* _onewayItem;
  UIBarButtonItem* _dotsItem;   ///< Vertex dots toggle; see _onewayItem.
  UIBarButtonItem* _namesItem;  ///< Place names toggle; see _onewayItem.
  UIBarButtonItem* _roadsItem;  ///< Road class menu button.
  /// Load error shown instead of the map, or nil.
  NSString* _error;
  /// Clock, frame numbers, touches and gesture names for screen recordings.
  VrstDebugOverlay* _debug;
  /// Vrst_session_log::appended() when the console was last fed.
  Vrst_uint64 _consoleSeen;
  /// [self now] when the console was last fed.
  double _consoleFedAt;
  /// View scale when the running pinch began, for its "xfactor" text.
  Vrst_float64 _pinchStartScale;
  /// Rotation of the running twist so far, in radians.
  Vrst_float64 _rotationSum;
  /// Vrst_tile_renderer::uploaded() after the previous frame.
  Vrst_uint64 _lastUploaded;
  /// Session log: frames, touches, gestures, tiles, stalls (Documents/logs).
  Vrst_session_log _log;
  /// CACurrentMediaTime() at launch; every `t` in the log and on the clock is relative to it.
  CFTimeInterval _t0;
  /// When the first redraw since the last frame was asked for; 0 = none pending.
  CFTimeInterval _queuedAt;
  /// UITouch.timestamp of the newest touch change seen.
  CFTimeInterval _lastTouchAt;
  /// Test scenario playing (or played); see scenario.h.
  std::optional<Vrst_scenario> _scenario;
  /// Scenario named by `--scenario`, started after the first frame; nil when none.
  NSString* _launchScenario;
  /// `--console`: open the console after the first frame.
  BOOL _launchConsole;
  /// `--record <s>`: record activity from the first frame for this long; 0 = not asked.
  double _launchRecordSeconds;
  /// [self now] when a `--record` recording stops; 0 = none running.
  double _recordStopAt;
  /// Scenario steps logged so far; a slow frame can pass several steps at once.
  Vrst_size _loggedSteps;
  /// `t` of the last "queue" record.
  double _lastQueueLog;
  /// The last "queue" record said settled: no more until loading starts again.
  BOOL _queueLoggedSettled;
  /// "stuck" records written: worker, piece and step text of each (one per area).
  std::set<std::string> _stuckDumped;
  /// Time on one piece after which a worker counts as stuck, ms (kStuckMs, or `--stuck-ms`).
  double _stuckMs;
  /// View size at the last layout pass; a redraw is needed only when it changes.
  CGSize _laidOutSize;
  /// CACurrentMediaTime() when a test scenario last moved the camera; 0 = never.
  CFTimeInterval _lastMoveAt;
  /// Pinch recognizer: may run together with _rotateRecognizer, nothing else.
  UIPinchGestureRecognizer* _pinchRecognizer;
  /// Two-finger twist recognizer; see _pinchRecognizer.
  UIRotationGestureRecognizer* _rotateRecognizer;
  /// GPS track being replayed (GPX); empty when none is loaded.
  Vrst_gpx_track _track;
  /// Name of the loaded track file, for the menu, the banner and the log.
  NSString* _trackName;
  /// The track still has to be uploaded to the GPU (needs the GL context: next draw).
  BOOL _trackDirty;
  /// Playback position in track seconds (0 = first point).
  double _trackTime;
  /// Playback speed: track seconds per real second (--speed, default 10).
  double _trackSpeed;
  /// Playback running.
  BOOL _trackPlaying;
  /// Camera path being recorded as a scenario (record button).
  Vrst_activity_recorder _recorder;
  /// Round record button: start, or stop and save.
  UIButton* _recordButton;
  /// Round play button: pick a recording to play, or stop playing.
  UIButton* _playButton;
  /// The camera follows the marker; a drag turns it off, the menu turns it on.
  BOOL _trackFollow;
  /// Track file named by `--gpx`, loaded and played after the map is ready; nil = none.
  NSString* _launchGpx;
  /// `t` of the last banner update of the playing track.
  double _trackBannerAt;
}

/// Uses a GLKView as the root view.
- (void)loadView {
  self.view = [[GLKView alloc] initWithFrame:UIScreen.mainScreen.bounds];
}

/// Creates the GL context, loads the map and builds the overlay and gestures.
- (void)viewDidLoad {
  [super viewDidLoad];
  _t0 = CACurrentMediaTime();
  [self openLog];
  [self writeExampleScenario];
  _startCenter = kDefaultPoint;
  _trackSpeed = 10;
  _startMpp = kDefaultMetresPerPoint;
  _stuckMs = kStuckMs;
  [self parseArguments];

  _context = [[EAGLContext alloc] initWithAPI:kEAGLRenderingAPIOpenGLES3];
  GLKView* v = (GLKView*)self.view;
  v.context = _context;
  v.delegate = self;
  v.drawableColorFormat = GLKViewDrawableColorFormatRGBA8888;
  v.drawableMultisample = GLKViewDrawableMultisample4X;
  self.paused = YES;  // draw on demand, not 60 times a second
  self.resumeOnDidBecomeActive = NO;
  [EAGLContext setCurrentContext:_context];

  try {
    CFTimeInterval step = CACurrentMediaTime();
    // Milliseconds since the previous step, for the "load" record.
    const auto lap = [&step] {
      const CFTimeInterval now = CACurrentMediaTime();
      const double ms = (now - step) * 1000;
      step = now;
      return ms;
    };
    NSString* dir = [NSBundle.mainBundle.resourcePath stringByAppendingPathComponent:@"map"];
    _map = open_map(dir.UTF8String, {.threads = 2, .record_events = true});  // events: "build" records
    _mapFrame = Vrst_map_frame(_map->extent());
    const double map_ms = lap();
    const std::vector<Vrst_map_place> places = _map->places();
    _labels.build(places, _mapFrame);
    NSString* fontDir = [NSBundle.mainBundle.resourcePath stringByAppendingPathComponent:@"fonts"];
    std::vector<std::string> fonts;
    for (const std::string_view f : kLabelFontFiles) fonts.push_back(std::string(fontDir.UTF8String) + "/" + std::string(f));
    _atlas.build(fonts, _labels.codepoints());
    _labels.measure(_atlas);
    const double labels_ms = lap();
    _showLabels = YES;
    _loaded = YES;
    _renderer.init_overlays();
    _renderer.set_label_atlas(_atlas);
    _tiles.init();
    _glReady = YES;
    _log.record("load", [self now])
        .num("map_ms", map_ms, 1)
        .num("labels_ms", labels_ms, 1)
        .num("gl_ms", lap(), 1)
        .count("places", places.size());
    for (const std::string& line : _map->describe()) _log.record("map", [self now]).text("info", line);
  } catch (const std::exception& e) {
    _error = [NSString stringWithFormat:@"cannot load map: %s", e.what()];
    _log.record("error", [self now]).text("what", e.what());
  }
  if (_launchGpx && _loaded) [self loadTrack:_launchGpx];
  [self buildOverlay];
  [self addGestures];
  [self startLocation];
}

/// Seconds since launch: the `t` of the log and of the overlay clock.
/// @return Seconds.
- (double)now {
  return CACurrentMediaTime() - _t0;
}

/// Opens the session log in Documents/logs (or the `--log-dir <dir>` launch argument, which
/// lets a Simulator run write straight into a Mac folder) and writes the launch record;
/// listens for memory warnings, background / foreground and thermal state changes.
- (void)openLog {
  NSString* docs = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES).firstObject;
  NSString* dir = [docs stringByAppendingPathComponent:@"logs"];
  NSArray<NSString*>* args = NSProcessInfo.processInfo.arguments;
  const NSUInteger at = [args indexOfObject:@"--log-dir"];
  if (at != NSNotFound && at + 1 < args.count) dir = args[at + 1];
  try {
    _log.open(std::string(dir.UTF8String), kKeepSessions);
  } catch (const std::exception& e) {
    NSLog(@"verst: session log off: %s", e.what());  // the app works without it
    return;
  }
  utsname u{};
  uname(&u);
  NSDictionary* info = NSBundle.mainBundle.infoDictionary;
  const CGSize screen = UIScreen.mainScreen.nativeBounds.size;
  _log.record("system", 0)
      .text("event", "launch")
      .text("device", u.machine)
      .text("ios", UIDevice.currentDevice.systemVersion.UTF8String)
      .text("app", [info[@"CFBundleShortVersionString"] UTF8String] ?: "")
      .text("build", [info[@"CFBundleVersion"] UTF8String] ?: "")
      .num("screen_w", screen.width, 0)
      .num("screen_h", screen.height, 0)
      .num("scale", UIScreen.mainScreen.scale, 1)
      .num("max_fps", static_cast<double>(UIScreen.mainScreen.maximumFramesPerSecond), 0)
      .num("thermal", static_cast<double>(NSProcessInfo.processInfo.thermalState), 0)
      .text("config", kBuildConfig)
      .text("arch", kArch);
  NSNotificationCenter* nc = NSNotificationCenter.defaultCenter;
  __weak VrstMapController* weak = self;
  [nc addObserverForName:UIApplicationDidEnterBackgroundNotification object:nil queue:nil
              usingBlock:^(NSNotification*) {
                [weak logSystem:"background"];
                [weak stopRecording];  // iOS may end the app in the background: keep what was recorded
              }];
  [nc addObserverForName:UIApplicationWillEnterForegroundNotification object:nil queue:nil
              usingBlock:^(NSNotification*) { [weak logSystem:"foreground"]; }];
  [nc addObserverForName:NSProcessInfoThermalStateDidChangeNotification object:nil queue:NSOperationQueue.mainQueue
              usingBlock:^(NSNotification*) { [weak logSystem:"thermal"]; }];
}

/// Logs a system event and writes the log to the file at once (the app may be
/// suspended right after going to the background).
/// @param event Event name.
- (void)logSystem:(const char*)event {
  _log.record("system", [self now])
      .text("event", event)
      .num("thermal", static_cast<double>(NSProcessInfo.processInfo.thermalState), 0);
  _log.flush();
}

/// Memory warning from iOS: logged, nothing else changes yet.
- (void)didReceiveMemoryWarning {
  [super didReceiveMemoryWarning];
  [self logSystem:"memory_warning"];
}

/// Releases the GL objects while the context is still alive.
- (void)dealloc {
  _map.reset();  // stop the map workers
  if (_glReady) {
    [EAGLContext setCurrentContext:_context];
    _renderer.destroy();
    _tiles.destroy();
  }
}

/// Light status bar text on the dark map.
- (UIStatusBarStyle)preferredStatusBarStyle {
  return UIStatusBarStyleLightContent;
}

/// Reads `--center <lat> <lon>` and `--mpp <metres>` from the launch arguments.
- (void)parseArguments {
  NSArray<NSString*>* a = NSProcessInfo.processInfo.arguments;
  for (NSUInteger i = 1; i < a.count; ++i) {
    if ([a[i] isEqualToString:@"--center"] && i + 2 < a.count) {
      const double lat = a[i + 1].doubleValue, lon = a[i + 2].doubleValue;
      if (std::abs(lat) <= 90 && std::abs(lon) <= 180)
        _startCenter = {lat, lon};
      i += 2;
    } else if ([a[i] isEqualToString:@"--gpx"] && i + 1 < a.count) {
      _launchGpx = a[i + 1];
      ++i;
    } else if ([a[i] isEqualToString:@"--speed"] && i + 1 < a.count) {
      if (a[i + 1].doubleValue > 0) _trackSpeed = a[i + 1].doubleValue;
      ++i;
    } else if ([a[i] isEqualToString:@"--record"] && i + 1 < a.count) {
      _launchRecordSeconds = std::max(0.0, a[i + 1].doubleValue);
      ++i;
    } else if ([a[i] isEqualToString:@"--stuck-ms"] && i + 1 < a.count) {
      if (a[i + 1].doubleValue > 0) _stuckMs = a[i + 1].doubleValue;
      ++i;
    } else if ([a[i] isEqualToString:@"--console"]) {
      _launchConsole = YES;
    } else if ([a[i] isEqualToString:@"--scenario"] && i + 1 < a.count) {
      _launchScenario = a[i + 1];
      ++i;
    } else if ([a[i] isEqualToString:@"--mpp"] && i + 1 < a.count) {
      if (a[i + 1].doubleValue > 0) _startMpp = a[i + 1].doubleValue;
      ++i;
    }
  }
}

// --- drawing -----------------------------------------------------------------

/// Framebuffer pixels per view point (3 on most iPhones).
- (CGFloat)pxPerPoint {
  return self.view.contentScaleFactor;
}

/// Drawable width in framebuffer pixels.
- (int)fbWidth {
  return std::max(1, int(((GLKView*)self.view).drawableWidth));
}

/// Drawable height in framebuffer pixels.
- (int)fbHeight {
  return std::max(1, int(((GLKView*)self.view).drawableHeight));
}

/// Asks for one redraw.
- (void)redraw {
  if (_queuedAt == 0) _queuedAt = CACurrentMediaTime();
  [self.view setNeedsDisplay];
}

/// Redraws after rotation or any other size change. Only then: a layout pass also runs
/// whenever a label text changes, and redrawing for it made every frame ask for the next.
- (void)viewDidLayoutSubviews {
  [super viewDidLayoutSubviews];
  const CGSize size = self.view.bounds.size;
  if (CGSizeEqualToSize(size, _laidOutSize)) return;
  _laidOutSize = size;
  [self redraw];
}

/// GLKView draw callback: sets the start view on the first frame, then draws.
/// @param view The GLKView, with its framebuffer bound.
/// @param rect Dirty rectangle; the whole map is always redrawn.
- (void)glkView:(GLKView*)view drawInRect:(CGRect)rect {
  (void)rect;
  const CFTimeInterval drawStart = CACurrentMediaTime();
  if (!_glReady) {
    glClearColor(kBackground[0], kBackground[1], kBackground[2], 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    return;
  }
  const int w = int(view.drawableWidth), h = int(view.drawableHeight);
  if (!_viewSet && w > 0 && h > 0) {
    _initialScale = _mapFrame.fit_scale(w, h);
    [self resetView];
    _viewSet = YES;
  }
  const CGFloat px = [self pxPerPoint];
  CFTimeInterval mark = drawStart;
  // Milliseconds since the previous phase, for the "frame" record.
  const auto phase = [&mark] {
    const CFTimeInterval now = CACurrentMediaTime();
    const double ms = (now - mark) * 1000;
    mark = now;
    return ms;
  };
  Vrst_tile_frame tf;
  const std::vector<Vrst_uint64> keys =
      select_tiles(_tiles, *_map, _mapFrame, _view, w, h, ++_frame, kTileBudget, tf);
  _mapStatus = _map->status();
  const double select_ms = phase();
  _opt.point_px = Vrst_float32(2.5 * px);
  _opt.px_ratio = px;
  _tiles.draw(_view, w, h, keys, _mapFrame.ground_metres_per_point(_view, px), _opt);
  [self drawTrackWidth:w height:h];
  const double tiles_ms = phase();
  if (_hasFix) _renderer.draw_position(_view, w, h, _mapFrame.to_world(_fix), Vrst_float32(16 * px));
  const double position_ms = phase();
  if (_showLabels)
    _renderer.draw_labels(_labels.layout(_view, w, h, [self pxPerPoint], _atlas), w, h,
                          Vrst_float32(2.0 * [self pxPerPoint]));
  const double labels_ms = phase();
  [self updateStats];
  const Vrst_uint64 uploaded = _tiles.uploaded();
  [_debug frameDrawnInMs:(CACurrentMediaTime() - drawStart) * 1000
                 visible:tf.visible
                 missing:tf.missing
                building:_mapStatus.queued + _mapStatus.working
             uploadBytes:static_cast<NSUInteger>(uploaded - _lastUploaded)];
  const double stats_ms = phase();
  // Keep drawing while tiles are loading so they appear as soon as they are built;
  // otherwise draw only on demand.
  self.paused = _mapStatus.settled;
  if (_launchScenario) {
    NSString* name = _launchScenario;
    _launchScenario = nil;
    [self startScenario:name];
  }
  if (_launchConsole) {
    _launchConsole = NO;
    [self onConsole];
  }
  if (_launchRecordSeconds > 0) {
    [self startRecording];
    _recordStopAt = [self now] + _launchRecordSeconds;
    _launchRecordSeconds = 0;
  }
  [self logFrame:tf
         start:drawStart
        phases:{select_ms, tiles_ms, position_ms, labels_ms, stats_ms}
      uploaded:uploaded - _lastUploaded];
  _lastUploaded = uploaded;
}

/// Writes the "frame" record and one "tile" record per uploaded piece.
/// @param tf What select_tiles() did.
/// @param start CACurrentMediaTime() when the draw callback began.
/// @param ms Phase times: tile select + uploads, tile draw, position, labels, stats.
/// @param bytes Tile bytes uploaded in this frame.
- (void)logFrame:(const Vrst_tile_frame&)tf
           start:(CFTimeInterval)start
          phases:(std::array<double, 5>)ms
        uploaded:(Vrst_uint64)bytes {
  const CFTimeInterval end = CACurrentMediaTime();
  double upload_ms = 0;
  for (const Vrst_tile_upload& u : tf.uploads) upload_ms += u.upload_ms;
  {
    Vrst_log_record r = _log.record("frame", end - _t0);
    r.count("n", _frame)
        .num("start", start - _t0, 4)
        .num("ms", (end - start) * 1000, 2)
        .num("select_ms", ms[0], 2)
        .num("tiles_ms", ms[1], 2)
        .num("position_ms", ms[2], 2)
        .num("labels_ms", ms[3], 2)
        .num("stats_ms", ms[4], 2)
        .num("upload_ms", upload_ms, 2)
        .count("uploads", tf.uploads.size())
        .count("upload_bytes", bytes)
        .count("evicted", tf.evicted)
        .num("evict_ms", tf.evict_ms, 2)
        .count("visible", tf.visible)
        .count("missing", tf.missing)
        .count("fallback", tf.fallback)
        .count("building", _mapStatus.queued + _mapStatus.working)
        .count("cached", _tiles.size())
        .count("draw_calls", static_cast<Vrst_uint64>(_tiles.draw_calls()))
        .count("labels", _showLabels ? _labels.placed() : 0)
        .num("scale", _view.scale, 6)
        .num("angle", _view.angle, 4)
        .num("cx", _view.center.x, 1)
        .num("cy", _view.center.y, 1)
        .flag("continuous", !self.paused);
    if (_scenario && _scenario->running()) r.count("step", _scenario->step_index());
    if (_queuedAt > 0) r.num("queued", _queuedAt - _t0, 4);
    if (_lastTouchAt > 0) r.num("touch", _lastTouchAt - _t0, 4);
    if (_lastMoveAt > 0) r.num("move", _lastMoveAt - _t0, 4);
    if (_trackPlaying) r.num("track_t", _trackTime, 1);
  }  // the frame record is appended here, before its tiles
  _queuedAt = 0;
  for (const Vrst_tile_upload& u : tf.uploads)
    _log.record("tile", end - _t0)
        .text("piece", _map->piece_name(u.key))
        .count("bytes", u.bytes)
        .num("build_ms", u.build_ms, 2)
        .count("worker", static_cast<Vrst_uint64>(u.worker))
        .num("upload_ms", u.upload_ms, 2);
}

/// Start view: _startCenter at _startMpp, or the whole map if it is outside the data.
- (void)resetView {
  _view = _mapFrame.start_view(_startCenter, _startMpp, [self pxPerPoint], [self fbWidth], [self fbHeight]);
}

/// Whole map, no rotation.
- (void)fitView {
  _view = {{0, 0}, _mapFrame.fit_scale([self fbWidth], [self fbHeight]), 0};
}

// --- overlay -------------------------------------------------------------------

/// Stats label at the top and toolbar at the bottom, both inside the safe area.
- (void)buildOverlay {
  _debug = [[VrstDebugOverlay alloc] initWithFrame:self.view.bounds];
  _debug.translatesAutoresizingMaskIntoConstraints = NO;
  _debug.startTime = _t0;
  __weak VrstMapController* weak = self;
  _debug.touchHandler = ^(UITouch* t, NSUInteger finger, BOOL ended) { [weak logTouch:t finger:finger ended:ended]; };
  _debug.tickHandler = ^(CFTimeInterval now, double gapMs) { [weak onTick:now gap:gapMs]; };
  [self.view addSubview:_debug];

  _stats = [[UILabel alloc] init];
  _stats.translatesAutoresizingMaskIntoConstraints = NO;
  _stats.numberOfLines = 0;
  _stats.font = [UIFont monospacedSystemFontOfSize:11 weight:UIFontWeightRegular];
  _stats.textColor = UIColor.whiteColor;
  _stats.backgroundColor = [UIColor colorWithWhite:0 alpha:0.45];
  _stats.layer.cornerRadius = 6;
  _stats.layer.masksToBounds = YES;
  [self.view addSubview:_stats];
  _stats.userInteractionEnabled = YES;
  [_stats addGestureRecognizer:[[UITapGestureRecognizer alloc] initWithTarget:self action:@selector(onStatsTap)]];

  UIToolbar* bar = [[UIToolbar alloc] init];
  bar.translatesAutoresizingMaskIntoConstraints = NO;
  bar.barStyle = UIBarStyleBlack;
  bar.translucent = YES;
  bar.tintColor = UIColor.whiteColor;
  UIBarButtonItem* fit = [[UIBarButtonItem alloc] initWithTitle:@"Fit" style:UIBarButtonItemStylePlain
                                                         target:self action:@selector(onFit)];
  UIBarButtonItem* start = [[UIBarButtonItem alloc] initWithTitle:@"Start" style:UIBarButtonItemStylePlain
                                                           target:self action:@selector(onStart)];
  _roadsItem = [[UIBarButtonItem alloc] initWithTitle:@"Roads" menu:[self roadsMenu]];
  _onewayItem = [[UIBarButtonItem alloc] initWithTitle:@"Oneway" style:UIBarButtonItemStylePlain
                                                target:self action:@selector(onOneway)];
  _dotsItem = [[UIBarButtonItem alloc] initWithTitle:@"Dots" style:UIBarButtonItemStylePlain
                                              target:self action:@selector(onDots)];
  _namesItem = [[UIBarButtonItem alloc] initWithTitle:@"Names" style:UIBarButtonItemStylePlain
                                               target:self action:@selector(onNames)];
  UIBarButtonItem* log = [[UIBarButtonItem alloc] initWithTitle:@"Log" style:UIBarButtonItemStylePlain
                                                         target:self action:@selector(onConsole)];
  log.tintColor = UIColor.systemOrangeColor;
  UIBarButtonItem* flex = [[UIBarButtonItem alloc] initWithBarButtonSystemItem:UIBarButtonSystemItemFlexibleSpace
                                                                        target:nil action:nil];
  bar.items = @[ fit, flex, start, flex, _roadsItem, flex, _onewayItem, flex, _dotsItem, flex, _namesItem, flex, log ];
  // Seven titles do not fit a 390 pt wide phone at the default 17 pt.
  NSDictionary* small = @{NSFontAttributeName : [UIFont systemFontOfSize:15]};
  for (UIBarButtonItem* item in bar.items) {
    [item setTitleTextAttributes:small forState:UIControlStateNormal];
    [item setTitleTextAttributes:small forState:UIControlStateHighlighted];
  }
  [self.view addSubview:bar];

  UIButton* locate = [self roundButton:@"location.fill" color:UIColor.systemBlueColor action:@selector(onLocate)];
  _recordButton = [self roundButton:@"record.circle" color:UIColor.systemRedColor action:@selector(onRecordButton)];
  _playButton = [self roundButton:@"play.fill" color:UIColor.systemGreenColor action:@selector(onPlayButton)];

  // Map: the provider's work. User: what the user did. Events: all but the per-frame
  // and per-touch records. All: everything.
  _console.filters = @[
    [VrstConsoleFilter filterNamed:@"Map" only:[NSSet setWithArray:@[ @"build", @"queue", @"load", @"map", @"error" ]]],
    [VrstConsoleFilter filterNamed:@"User" only:[NSSet setWithArray:@[ @"touch", @"gesture", @"ui" ]]],
    [VrstConsoleFilter filterNamed:@"Events" except:[NSSet setWithArray:@[ @"frame", @"tile", @"touch", @"queue" ]]],
    [VrstConsoleFilter filterNamed:@"All" except:[NSSet set]],
  ];
  _console.closeHandler = ^{ [weak onConsoleClosed]; };

  UILayoutGuide* safe = self.view.safeAreaLayoutGuide;
  [NSLayoutConstraint activateConstraints:@[
    [_stats.topAnchor constraintEqualToAnchor:safe.topAnchor constant:6],
    [_stats.leadingAnchor constraintEqualToAnchor:safe.leadingAnchor constant:8],
    [_debug.topAnchor constraintEqualToAnchor:self.view.topAnchor],
    [_debug.bottomAnchor constraintEqualToAnchor:self.view.bottomAnchor],
    [_debug.leadingAnchor constraintEqualToAnchor:self.view.leadingAnchor],
    [_debug.trailingAnchor constraintEqualToAnchor:self.view.trailingAnchor],
    [_debug.panel.topAnchor constraintEqualToAnchor:_stats.bottomAnchor constant:6],
    [_debug.panel.leadingAnchor constraintEqualToAnchor:_stats.leadingAnchor],
    [bar.leadingAnchor constraintEqualToAnchor:self.view.leadingAnchor],
    [bar.trailingAnchor constraintEqualToAnchor:self.view.trailingAnchor],
    [bar.bottomAnchor constraintEqualToAnchor:safe.bottomAnchor],
    [locate.trailingAnchor constraintEqualToAnchor:safe.trailingAnchor constant:-12],
    [locate.bottomAnchor constraintEqualToAnchor:bar.topAnchor constant:-12],
    [_recordButton.trailingAnchor constraintEqualToAnchor:locate.trailingAnchor],
    [_recordButton.bottomAnchor constraintEqualToAnchor:locate.topAnchor constant:-12],
    [_playButton.trailingAnchor constraintEqualToAnchor:locate.trailingAnchor],
    [_playButton.bottomAnchor constraintEqualToAnchor:_recordButton.topAnchor constant:-12],
  ]];
  [self updateToggles];
  [self updateStats];
}

/// Menu with one checkable entry per road class group.
/// @return Menu reflecting the current class mask.
- (UIMenu*)roadsMenu {
  NSMutableArray<UIAction*>* items = [NSMutableArray array];
  for (int i = 0; i < kClassGroupCount; ++i) {
    __weak VrstMapController* weak = self;
    UIAction* a = [UIAction actionWithTitle:@(std::string(kGroupNames[i]).c_str()) image:nil identifier:nil
                                    handler:^(__kindof UIAction*) { [weak toggleGroup:i]; }];
    a.state = (_opt.class_mask & kClassGroups[i]) ? UIMenuElementStateOn : UIMenuElementStateOff;
    [items addObject:a];
  }
  return [UIMenu menuWithTitle:@"Road classes" children:items];
}

/// Shows or hides one road class group.
/// @param group Index into kClassGroups.
- (void)toggleGroup:(int)group {
  _opt.class_mask ^= kClassGroups[group];
  _roadsItem.menu = [self roadsMenu];
  [self redraw];
}

/// Tints the toggle buttons: pink when on, grey when off.
- (void)updateToggles {
  _onewayItem.tintColor = _opt.oneway ? UIColor.systemPinkColor : UIColor.lightGrayColor;
  _dotsItem.tintColor = _opt.show_vertices ? UIColor.systemPinkColor : UIColor.lightGrayColor;
  _namesItem.tintColor = _showLabels ? UIColor.systemPinkColor : UIColor.lightGrayColor;
}

/// Refreshes the stats line: counts, zoom and ground resolution, or the load error.
- (void)updateStats {
  if (_error) {
    _stats.text = [NSString stringWithFormat:@" %@ ", _error];
    return;
  }
  if (!_viewSet) return;
  const Vrst_float64 lat = inverse_mercator(_mapFrame.center()).lat * kPi / 180;
  NSString* gps = _hasFix ? [NSString stringWithFormat:@"GPS %.5f, %.5f  +-%.0f m", _fix.lat, _fix.lon, _fixAccuracy]
                          : @"GPS waiting";
  const std::vector<std::string> info = _map->describe();
  std::vector<std::string> status = status_lines(_mapStatus, *_map);
  if (_mapStatus.settled) status.resize(1);  // "loading done"; idle workers say nothing
  std::string work;
  for (const std::string& line : status) work += " " + line + " \n";
  NSString* rec = _recorder.recording()
                     ? [NSString stringWithFormat:@" REC %.0f s \n", _recorder.seconds([self now])]
                     : @"";
  _stats.text = [NSString stringWithFormat:@"%@ %s \n zoom %.2fx  %.2f m/pt \n%s %@ ",
                 rec, info.empty() ? "" : info.front().c_str(), _view.scale / _initialScale,
                 [self pxPerPoint] * std::cos(lat) / _view.scale, work.c_str(), gps];
}

/// Toolbar: opens the console above the map (Split), fed from the start of the kept
/// log; closes it when it is open.
- (void)onConsole {
  if (_console.shown) {
    [_console close];
    return;
  }
  _log.record("ui", [self now]).text("event", "console_open");
  [_console clearLog];
  const Vrst_uint64 kept = _log.tail().size();
  _consoleSeen = _log.appended() - kept;
  [self feedConsoleAt:[self now]];
  [_console show:VrstConsoleLayoutSplit];
}

/// The console closed: logged, so a recording shows when the map got the screen back.
- (void)onConsoleClosed {
  _log.record("ui", [self now]).text("event", "console_closed");
}

/// Gives the console the provider status and the log lines written since the last call.
/// @param t Seconds since launch.
- (void)feedConsoleAt:(double)t {
  _consoleFedAt = t;
  NSMutableArray<NSString*>* status = [NSMutableArray array];
  if (_map) {
    for (const std::string& line : _map->describe()) [status addObject:@(line.c_str())];
    for (const std::string& line : status_lines(_map->status(), *_map)) [status addObject:@(line.c_str())];
  } else if (_error) {
    [status addObject:_error];
  }
  [_console setStatusLines:status];
  const std::deque<std::string>& tail = _log.tail();
  const Vrst_size fresh = static_cast<Vrst_size>(std::min<Vrst_uint64>(_log.appended() - _consoleSeen, tail.size()));
  _consoleSeen = _log.appended();
  NSMutableArray<NSString*>* lines = [NSMutableArray arrayWithCapacity:fresh];
  for (Vrst_size i = tail.size() - fresh; i < tail.size(); ++i) {
    NSString* line = @(readable_line(tail[i]).c_str());
    if (line) [lines addObject:line];  // nil for text that is not UTF-8
  }
  [_console appendLogLines:lines];
}

/// Toolbar: whole map.
- (void)onFit {
  [_debug showGesture:@"BUTTON Fit"];
  _log.record("ui", [self now]).text("event", "button_fit");
  [self fitView];
  [self redraw];
}

/// Toolbar: start view.
- (void)onStart {
  [_debug showGesture:@"BUTTON Start"];
  _log.record("ui", [self now]).text("event", "button_start");
  [self resetView];
  [self redraw];
}

/// Round button: puts the GPS position in the screen center, keeping zoom and
/// rotation, and stops following a replayed track (it would pull the camera back).
- (void)onLocate {
  if (!_hasFix) {
    [_debug showGesture:@"BUTTON Locate: GPS waiting"];
    _log.record("ui", [self now]).text("event", "button_locate").flag("fix", false);
    return;
  }
  [_debug showGesture:@"BUTTON Locate"];
  _log.record("ui", [self now])
      .text("event", "button_locate")
      .flag("fix", true)
      .num("lat", _fix.lat, 6)
      .num("lon", _fix.lon, 6);
  _trackFollow = NO;
  _view.center = _mapFrame.to_world(_fix);
  [self redraw];
}

/// Toolbar: oneway highlight.
- (void)onOneway {
  _opt.oneway = !_opt.oneway;
  [_debug showGesture:_opt.oneway ? @"BUTTON Oneway on" : @"BUTTON Oneway off"];
  [self updateToggles];
  [self redraw];
}

/// Toolbar: vertex dots.
- (void)onDots {
  _opt.show_vertices = !_opt.show_vertices;
  [_debug showGesture:_opt.show_vertices ? @"BUTTON Dots on" : @"BUTTON Dots off"];
  [self updateToggles];
  [self redraw];
}

/// Logs one touch change.
/// @param t The touch.
/// @param finger Finger number (1 = first finger down).
/// @param ended YES when the finger lifted or the touch was cancelled.
- (void)logTouch:(UITouch*)t finger:(NSUInteger)finger ended:(BOOL)ended {
  const CGPoint p = [t locationInView:self.view];
  _lastTouchAt = std::max(_lastTouchAt, t.timestamp);
  if (t.phase == UITouchPhaseBegan && _scenario && _scenario->running()) {
    _scenario->stop();  // the user takes over
    UIApplication.sharedApplication.idleTimerDisabled = NO;
    _log.record("scenario", [self now]).text("event", "cancelled").text("name", _scenario->name());
    [_debug showGesture:@"TEST stopped by touch"];
  }
  static constexpr const char* kPhase[] = {"began", "moved", "stationary", "ended", "cancelled"};
  const auto phase = static_cast<std::size_t>(t.phase);
  _log.record("touch", [self now])
      .count("finger", finger)
      .text("phase", ended ? "ended" : (phase < std::size(kPhase) ? kPhase[phase] : "other"))
      .num("x", p.x, 1)
      .num("y", p.y, 1)
      .num("touch", t.timestamp - _t0, 4);
}

/// Display-link tick: logs main-thread stalls and writes the log about once a second.
/// @param now CACurrentMediaTime() of the tick.
/// @param gapMs Time since the previous tick in ms.
- (void)onTick:(CFTimeInterval)now gap:(double)gapMs {
  if (gapMs > kStallMs) _log.record("stall", now - _t0).num("gap_ms", gapMs, 1);
  if (_scenario && _scenario->running() && _viewSet) [self playScenarioAt:now - _t0];
  if (_trackPlaying && _viewSet) [self playTrackFor:gapMs / 1000];
  if (_map) [self logLoaderAt:now - _t0];
  if (_recorder.recording() && _viewSet) _recorder.sample(now - _t0, _view);
  [self updateRecordButtons];
  if (_recordStopAt > 0 && now - _t0 >= _recordStopAt) {
    _recordStopAt = 0;
    [self stopRecording];
  }
  if (_console.shown && now - _t0 - _consoleFedAt >= kConsoleSeconds) [self feedConsoleAt:now - _t0];
  _log.flush_if_due(now - _t0);
}

/// Logs the map provider's work: every event since the last tick ("build": worker
/// start / done / failed, and "settled" when everything on screen is loaded), and a
/// snapshot of the queue and the workers 4 times a second while pieces load, plus one
/// when loading is done ("queue"); idle snapshots would only repeat themselves.
/// @param t Seconds since launch.
- (void)logLoaderAt:(double)t {
  static constexpr const char* kKind[] = {"start", "done", "failed", "settled"};
  for (const Vrst_work_event& e : _map->take_events()) {
    Vrst_log_record r = _log.record("build", t - e.age_ms / 1000);
    r.text("event", kKind[static_cast<int>(e.kind)]);
    if (e.kind == Vrst_work_event_kind::Settled) continue;
    r.count("worker", static_cast<Vrst_uint64>(e.worker)).text("piece", _map->piece_name(e.key));
    if (e.kind != Vrst_work_event_kind::Started) r.num("ms", e.ms, 2);
    if (!e.detail.empty()) r.text("detail", e.detail);
  }
  if (t - _lastQueueLog < 0.25) return;
  const Vrst_provider_status s = _map->status();
  if (s.settled && _queueLoggedSettled) return;
  _lastQueueLog = t;
  _queueLoggedSettled = s.settled;
  Vrst_log_record r = _log.record("queue", t);
  r.count("threads", s.threads)
      .count("queued", s.queued)
      .count("building", s.working)
      .count("done", s.ready)
      .count("dropped", s.dropped_total)
      .count("built", s.done_total)
      .count("failed", s.failed_total)
      .count("missing", s.missing)
      .flag("settled", s.settled);
  for (Vrst_size w = 0; w < s.workers.size(); ++w) {
    const Vrst_worker_info& ws = s.workers[w];
    const std::string key = "w" + std::to_string(w);
    r.text(key, ws.busy ? _map->piece_name(ws.key) : "idle");
    if (ws.busy) r.num(key + "_ms", ws.busy_ms, 0).text(key + "_step", ws.step);
  }
  [self logStuckWorkers:s at:t];
}

/// Writes a "stuck" record for each worker that has been on one piece longer than
/// kStuckMs: the piece, its step, the slowest area so far and, when it is triangulating,
/// the exact input ring, so the build can be repeated on the Mac. Once per worker,
/// piece and area; at most kMaxStuckDumps per session.
/// @param s Provider status.
/// @param t Seconds since launch.
- (void)logStuckWorkers:(const Vrst_provider_status&)s at:(double)t {
  for (Vrst_size w = 0; w < s.workers.size(); ++w) {
    const Vrst_worker_info& ws = s.workers[w];
    if (!ws.busy || ws.busy_ms < _stuckMs || _stuckDumped.size() >= kMaxStuckDumps) continue;
    // The step text without its running time, so one area is dumped once.
    const std::string step = ws.step.substr(0, ws.step.find(" pts "));
    const std::string id = std::to_string(w) + " " + _map->piece_name(ws.key) + " " + step;
    if (!_stuckDumped.insert(id).second) continue;
    Vrst_log_record r = _log.record("stuck", t);
    r.count("worker", w).text("piece", _map->piece_name(ws.key)).num("busy_ms", ws.busy_ms, 0);
    for (const std::string& line : _map->work_dump(static_cast<int>(w))) {
      const Vrst_size colon = line.find(": ");
      if (colon != std::string::npos) r.text(line.substr(0, colon), line.substr(colon + 2));
    }
  }
}

/// Logs a gesture event.
/// @param kind Gesture name, e.g. "pinch".
/// @param g Its recognizer, for the state.
/// @param value Gesture value (zoom factor so far, speed, degrees) or NAN for none.
- (void)logGesture:(const char*)kind recognizer:(UIGestureRecognizer*)g value:(double)value {
  static constexpr const char* kState[] = {"possible", "began", "changed", "ended", "cancelled", "failed"};
  const auto state = static_cast<std::size_t>(g.state);
  Vrst_log_record r = _log.record("gesture", [self now]);
  r.text("kind", kind).text("state", state < std::size(kState) ? kState[state] : "other").count("touches",
                                                                                                    g.numberOfTouches);
  if (!std::isnan(value)) r.num("value", value, 3);
}

/// Tap on the stats box: debug menu (overlay, send logs, delete logs).
- (void)onStatsTap {
  _log.flush();
  const std::vector<std::filesystem::path> files = _log.files();
  NSString* sizes = [NSString stringWithFormat:@"%lu sessions, %.1f MB", (unsigned long)files.size(),
                                               static_cast<double>(_log.total_bytes()) / 1048576.0];
  UIAlertController* menu = [UIAlertController alertControllerWithTitle:@"Debug" message:sizes
                                                         preferredStyle:UIAlertControllerStyleActionSheet];
  __weak VrstMapController* weak = self;
  [menu addAction:[UIAlertAction actionWithTitle:_debug.hidden ? @"Show overlay" : @"Hide overlay"
                                           style:UIAlertActionStyleDefault
                                         handler:^(UIAlertAction*) { [weak toggleOverlay]; }]];
  [menu addAction:[UIAlertAction actionWithTitle:@"Send this session..." style:UIAlertActionStyleDefault
                                         handler:^(UIAlertAction*) { [weak shareLogs:NO]; }]];
  [menu addAction:[UIAlertAction actionWithTitle:[NSString stringWithFormat:@"Send all sessions (%lu)...",
                                                                            (unsigned long)files.size()]
                                           style:UIAlertActionStyleDefault
                                         handler:^(UIAlertAction*) { [weak shareLogs:YES]; }]];
  if (_scenario && _scenario->running()) {
    [menu addAction:[UIAlertAction actionWithTitle:@"Stop test" style:UIAlertActionStyleDefault
                                           handler:^(UIAlertAction*) { [weak stopScenario]; }]];
  } else {
    for (NSString* name in [self scenarioNames])
      [menu addAction:[UIAlertAction actionWithTitle:[([name hasPrefix:kRecordingPrefix] ? @"Play recording: " : @"Run test: ")
                                                         stringByAppendingString:name]
                                               style:UIAlertActionStyleDefault
                                             handler:^(UIAlertAction*) { [weak startScenario:name]; }]];
  }
  if (_trackPlaying) {
    [menu addAction:[UIAlertAction actionWithTitle:@"Stop track" style:UIAlertActionStyleDefault
                                           handler:^(UIAlertAction*) { [weak stopTrack]; }]];
  } else if (!_track.empty()) {
    [menu addAction:[UIAlertAction actionWithTitle:[@"Resume track: " stringByAppendingString:_trackName]
                                             style:UIAlertActionStyleDefault
                                           handler:^(UIAlertAction*) { [weak resumeTrack]; }]];
  }
  if (!_track.empty() && !_trackFollow)
    [menu addAction:[UIAlertAction actionWithTitle:@"Follow track" style:UIAlertActionStyleDefault
                                           handler:^(UIAlertAction*) { [weak followTrack]; }]];
  for (NSString* name in [self trackNames])
    [menu addAction:[UIAlertAction actionWithTitle:[@"Play track: " stringByAppendingString:name]
                                             style:UIAlertActionStyleDefault
                                           handler:^(UIAlertAction*) { [weak loadTrack:name]; }]];
  [menu addAction:[UIAlertAction actionWithTitle:@"Delete older sessions" style:UIAlertActionStyleDestructive
                                         handler:^(UIAlertAction*) { [weak deleteOldLogs]; }]];
  [menu addAction:[UIAlertAction actionWithTitle:@"Cancel" style:UIAlertActionStyleCancel handler:nil]];
  menu.popoverPresentationController.sourceView = _stats;
  [self presentViewController:menu animated:YES completion:nil];
}

// --- GPS track replay -------------------------------------------------------------

/// Directory of GPX tracks: Documents/tracks (visible in the Files app; devicectl copies
/// files there).
/// @return Path.
- (NSString*)trackDir {
  NSString* docs = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES).firstObject;
  return [docs stringByAppendingPathComponent:@"tracks"];
}

/// GPX files in the track directory, by name.
/// @return File names.
- (NSArray<NSString*>*)trackNames {
  NSMutableArray<NSString*>* names = [NSMutableArray array];
  for (NSString* f in [[NSFileManager.defaultManager contentsOfDirectoryAtPath:[self trackDir] error:nil]
           sortedArrayUsingSelector:@selector(compare:)])
    if ([f.pathExtension.lowercaseString isEqualToString:@"gpx"]) [names addObject:f];
  return names;
}

/// Loads a GPX track (a file in the track directory, or a full path), puts the camera on
/// its start and starts playing it, following the marker. A bad file is shown and logged.
/// @param name Track name.
- (void)loadTrack:(NSString*)name {
  NSString* path = name.absolutePath ? name : [[self trackDir] stringByAppendingPathComponent:name];
  try {
    Vrst_gpx_track t;
    t.load(path.UTF8String);
    if (t.empty()) throw std::runtime_error("no track points");
    _track = std::move(t);
  } catch (const std::exception& e) {
    _log.record("track", [self now]).text("event", "error").text("what", e.what());
    [_debug showGesture:@"TRACK error (see log)"];
    return;
  }
  _trackName = name.lastPathComponent;
  _trackDirty = YES;
  _trackTime = 0;
  _trackPlaying = YES;
  _trackFollow = YES;
  _trackBannerAt = 0;
  _startCenter = _track.points().front();  // used by the first draw's start view
  _view.center = _mapFrame.to_world(_track.points().front());
  _log.record("track", [self now])
      .text("event", "play")
      .text("name", _trackName.UTF8String)
      .count("points", _track.points().size())
      .num("duration_s", _track.duration(), 1)
      .num("length_km", _track.length_m() / 1000, 2)
      .num("speed", _trackSpeed, 1);
  [_debug showGesture:[NSString stringWithFormat:@"TRACK %@  x%.0f", _trackName, _trackSpeed]];
  [self redraw];
}

/// Advances playback by real time and moves the camera to the marker while following.
/// @param seconds Real seconds since the previous display tick.
- (void)playTrackFor:(double)seconds {
  _trackTime = std::min(_trackTime + std::min(seconds, 0.25) * _trackSpeed, _track.duration());
  const Vrst_track_sample ts = _track.sample(_trackTime);
  if (_trackFollow) _view.center = _mapFrame.to_world(ts.pos);
  const double t = [self now];
  if (t - _trackBannerAt > 1) {
    _trackBannerAt = t;
    const int at = static_cast<int>(_trackTime), total = static_cast<int>(_track.duration());
    [_debug updateGesture:[NSString stringWithFormat:@"TRACK %d:%02d / %d:%02d  x%.0f  %.0f km/h", at / 60, at % 60,
                                                     total / 60, total % 60, _trackSpeed, ts.speed_kmh]];
  }
  if (_trackTime >= _track.duration()) {
    _trackPlaying = NO;
    _log.record("track", t).text("event", "finished").text("name", _trackName.UTF8String);
    [_debug showGesture:@"TRACK finished"];
  }
  [self redraw];
}

/// Uploads a newly loaded track, then draws the route, the driven part and the marker.
/// @param w Framebuffer width in px.
/// @param h Framebuffer height in px.
- (void)drawTrackWidth:(int)w height:(int)h {
  if (_track.empty()) return;
  if (_trackDirty) {
    std::vector<Vrst_vec2d> world;
    world.reserve(_track.points().size());
    for (const Vrst_latlon& c : _track.points()) world.push_back(_mapFrame.to_world(c));
    _renderer.set_track(world);
    _trackDirty = NO;
  }
  const Vrst_track_sample ts = _track.sample(_trackTime);
  _renderer.draw_track(_view, w, h, ts.index + 1, _mapFrame.to_world(ts.pos),
                       static_cast<Vrst_float32>(14 * [self pxPerPoint]));
}

/// Menu: pauses playback.
- (void)stopTrack {
  _trackPlaying = NO;
  _log.record("track", [self now]).text("event", "stopped").num("track_t", _trackTime, 1);
  [_debug showGesture:@"TRACK paused"];
}

/// Menu: continues playback (from the start when it had finished).
- (void)resumeTrack {
  if (_trackTime >= _track.duration()) _trackTime = 0;
  _trackPlaying = YES;
  _log.record("track", [self now]).text("event", "resumed").num("track_t", _trackTime, 1);
  [self redraw];
}

/// Menu: the camera follows the marker again.
- (void)followTrack {
  _trackFollow = YES;
  _log.record("track", [self now]).text("event", "follow_on");
  [self redraw];
}

// --- test scenarios ----------------------------------------------------------------

/// Directory of user scenario files: Documents/scenarios (visible in the Files app).
/// @return Path.
- (NSString*)scenarioDir {
  NSString* docs = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES).firstObject;
  return [docs stringByAppendingPathComponent:@"scenarios"];
}

/// Writes the built-in scenario as `standard-example.txt` into the scenario directory,
/// once, so there is a file to copy and edit in the Files app.
- (void)writeExampleScenario {
  NSString* dir = [self scenarioDir];
  NSString* file = [dir stringByAppendingPathComponent:@"standard-example.txt"];
  NSFileManager* fm = NSFileManager.defaultManager;
  if ([fm fileExistsAtPath:file]) return;
  [fm createDirectoryAtPath:dir withIntermediateDirectories:YES attributes:nil error:nil];
  NSString* text = [[NSString alloc] initWithBytes:kStandardScenario.data()
                                            length:kStandardScenario.size()
                                          encoding:NSUTF8StringEncoding];
  [text writeToFile:file atomically:YES encoding:NSUTF8StringEncoding error:nil];
}

/// Directory of the scenarios shipped with the app (ios/scenarios in the repo).
/// @return Path inside the app bundle.
- (NSString*)bundledScenarioDir {
  return [NSBundle.mainBundle.resourcePath stringByAppendingPathComponent:@"scenarios"];
}

/// Scenarios the menu offers: "standard" (built in), the bundled ones by name, then every
/// .txt the user put in Documents/scenarios, by file name.
/// @return Names for startScenario:.
- (NSArray<NSString*>*)scenarioNames {
  NSMutableArray<NSString*>* names = [NSMutableArray arrayWithObject:@"standard"];
  NSFileManager* fm = NSFileManager.defaultManager;
  for (NSString* f in [[fm contentsOfDirectoryAtPath:[self bundledScenarioDir] error:nil]
           sortedArrayUsingSelector:@selector(compare:)])
    if ([f.pathExtension isEqualToString:@"txt"]) [names addObject:f.stringByDeletingPathExtension];
  for (NSString* f in [[fm contentsOfDirectoryAtPath:[self scenarioDir] error:nil]
           sortedArrayUsingSelector:@selector(compare:)])
    if ([f.pathExtension isEqualToString:@"txt"]) [names addObject:f];
  return names;
}

/// File of a scenario name: a bundled `<name>.txt`, a file in Documents/scenarios, or a
/// full path.
/// @param name Scenario name.
/// @return Path; the file may not exist.
- (NSString*)scenarioPath:(NSString*)name {
  if (name.absolutePath) return name;
  NSString* bundled = [[self bundledScenarioDir] stringByAppendingPathComponent:[name stringByAppendingPathExtension:@"txt"]];
  if ([NSFileManager.defaultManager fileExistsAtPath:bundled]) return bundled;
  return [[self scenarioDir] stringByAppendingPathComponent:name];
}

/// A round button on the map, added to the view; the caller places it.
/// @param symbol SF Symbol name.
/// @param color Icon color.
/// @param action Selector called on tap.
/// @return The button, kRoundButtonSize wide and high.
- (UIButton*)roundButton:(NSString*)symbol color:(UIColor*)color action:(SEL)action {
  UIButton* b = [UIButton buttonWithType:UIButtonTypeSystem];
  b.translatesAutoresizingMaskIntoConstraints = NO;
  [b setImage:[UIImage systemImageNamed:symbol] forState:UIControlStateNormal];
  b.tintColor = color;
  b.backgroundColor = [UIColor colorWithWhite:0.1 alpha:0.8];
  b.layer.cornerRadius = kRoundButtonSize / 2;
  [b addTarget:self action:action forControlEvents:UIControlEventTouchUpInside];
  [self.view addSubview:b];
  [NSLayoutConstraint activateConstraints:@[
    [b.widthAnchor constraintEqualToConstant:kRoundButtonSize],
    [b.heightAnchor constraintEqualToConstant:kRoundButtonSize],
  ]];
  return b;
}

/// Shows on the record and play buttons what a tap does now: record, or stop and save;
/// play, or stop playing. Cheap when nothing changed (called on every display tick).
- (void)updateRecordButtons {
  NSString* recIcon = _recorder.recording() ? @"stop.circle.fill" : @"record.circle";
  NSString* playIcon = _scenario && _scenario->running() ? @"stop.fill" : @"play.fill";
  if (![_recordButton.accessibilityIdentifier isEqualToString:recIcon]) {
    [_recordButton setImage:[UIImage systemImageNamed:recIcon] forState:UIControlStateNormal];
    _recordButton.accessibilityIdentifier = recIcon;
  }
  if (![_playButton.accessibilityIdentifier isEqualToString:playIcon]) {
    [_playButton setImage:[UIImage systemImageNamed:playIcon] forState:UIControlStateNormal];
    _playButton.accessibilityIdentifier = playIcon;
  }
}

/// Record button: starts recording, or stops and saves the recording.
- (void)onRecordButton {
  if (_recorder.recording()) {
    [self stopRecording];
  } else {
    [self startRecording];
  }
}

/// Play button: while a recording or test plays, stops it; otherwise offers the saved
/// recordings, newest first (each opens chooseRecording:).
- (void)onPlayButton {
  if (_scenario && _scenario->running()) {
    [self stopScenario];
    return;
  }
  NSMutableArray<NSString*>* recs = [NSMutableArray array];
  for (NSString* name in [self scenarioNames].reverseObjectEnumerator)
    if ([name hasPrefix:kRecordingPrefix]) [recs addObject:name];
  if (recs.count == 0) {
    [_debug showGesture:@"No recordings yet: tap the red button"];
    return;
  }
  UIAlertController* menu = [UIAlertController alertControllerWithTitle:@"Play recording" message:nil
                                                         preferredStyle:UIAlertControllerStyleActionSheet];
  __weak VrstMapController* weak = self;
  for (NSString* name in recs)
    [menu addAction:[UIAlertAction actionWithTitle:name style:UIAlertActionStyleDefault
                                           handler:^(UIAlertAction*) { [weak chooseRecording:name]; }]];
  [menu addAction:[UIAlertAction actionWithTitle:@"Cancel" style:UIAlertActionStyleCancel handler:nil]];
  menu.popoverPresentationController.sourceView = _playButton;
  [self presentViewController:menu animated:YES completion:nil];
}

/// What to do with one recording: play it or share the file.
/// @param name Recording file name in Documents/scenarios.
- (void)chooseRecording:(NSString*)name {
  UIAlertController* menu = [UIAlertController alertControllerWithTitle:name message:nil
                                                         preferredStyle:UIAlertControllerStyleActionSheet];
  __weak VrstMapController* weak = self;
  [menu addAction:[UIAlertAction actionWithTitle:@"Play" style:UIAlertActionStyleDefault
                                         handler:^(UIAlertAction*) { [weak startScenario:name]; }]];
  [menu addAction:[UIAlertAction actionWithTitle:@"Share..." style:UIAlertActionStyleDefault
                                         handler:^(UIAlertAction*) { [weak shareRecording:name]; }]];
  [menu addAction:[UIAlertAction actionWithTitle:@"Cancel" style:UIAlertActionStyleCancel handler:nil]];
  menu.popoverPresentationController.sourceView = _playButton;
  [self presentViewController:menu animated:YES completion:nil];
}

/// Opens the share sheet (AirDrop, Mail, Messages, Files ...) with a recording file.
/// @param name Recording file name in Documents/scenarios.
- (void)shareRecording:(NSString*)name {
  _log.record("ui", [self now]).text("event", "share_recording").text("file", name.UTF8String);
  NSURL* url = [NSURL fileURLWithPath:[[self scenarioDir] stringByAppendingPathComponent:name]];
  UIActivityViewController* share = [[UIActivityViewController alloc] initWithActivityItems:@[ url ]
                                                                      applicationActivities:nil];
  share.popoverPresentationController.sourceView = _playButton;
  [self presentViewController:share animated:YES completion:nil];
}

/// Starts recording the camera path (whatever moves the map); a playing test or
/// recording is stopped first, so only the user's own moves are recorded.
- (void)startRecording {
  if (!_viewSet || _recorder.recording()) return;
  if (_scenario && _scenario->running()) [self stopScenario];
  _recorder.start([self now], _view, _mapFrame.center(), [self pxPerPoint]);
  _log.record("ui", [self now]).text("event", "record_start");
  [_debug showGesture:@"REC started"];
  [self redraw];
}

/// Ends the recording and saves it as `rec-<YYYYMMDD-HHMMSS>.txt` in Documents/scenarios,
/// where the play button offers it. Does nothing when not recording.
- (void)stopRecording {
  if (!_recorder.recording()) return;
  const double seconds = _recorder.seconds([self now]);
  const std::string text = _recorder.finish([self now]);
  NSDateFormatter* fmt = [[NSDateFormatter alloc] init];
  fmt.dateFormat = @"yyyyMMdd-HHmmss";
  NSString* name = [NSString stringWithFormat:@"%@%@.txt", kRecordingPrefix, [fmt stringFromDate:NSDate.date]];
  NSString* dir = [self scenarioDir];
  [NSFileManager.defaultManager createDirectoryAtPath:dir withIntermediateDirectories:YES attributes:nil error:nil];
  NSString* body = [[NSString alloc] initWithBytes:text.data() length:text.size() encoding:NSUTF8StringEncoding];
  const BOOL saved = [body writeToFile:[dir stringByAppendingPathComponent:name] atomically:YES
                              encoding:NSUTF8StringEncoding error:nil];
  _log.record("ui", [self now])
      .text("event", "record_stop")
      .text("file", name.UTF8String)
      .num("seconds", seconds, 1)
      .flag("saved", saved);
  [_debug showGesture:saved ? [NSString stringWithFormat:@"SAVED %@ (%.0f s)", name, seconds] : @"REC not saved"];
  [self redraw];
}

/// Starts a scenario: "standard", a file in the scenario directory, or a full path.
/// A bad file is reported on screen and in the log, and nothing starts.
/// @param name Scenario name.
- (void)startScenario:(NSString*)name {
  try {
    if ([name isEqualToString:@"standard"]) {
      _scenario = Vrst_scenario::parse("standard", kStandardScenario);
    } else {
      NSString* path = [self scenarioPath:name];
      NSString* text = [NSString stringWithContentsOfFile:path encoding:NSUTF8StringEncoding error:nil];
      if (!text) throw std::runtime_error(std::string("cannot read scenario ") + path.UTF8String);
      _scenario = Vrst_scenario::parse(name.UTF8String, text.UTF8String);
    }
  } catch (const std::exception& e) {
    _log.record("scenario", [self now]).text("event", "error").text("what", e.what());
    [_debug showGesture:@"TEST error (see log)"];
    return;
  }
  _scenario->start([self now]);
  _loggedSteps = 0;
  UIApplication.sharedApplication.idleTimerDisabled = YES;  // no auto-lock while the test plays
  _log.record("scenario", [self now])
      .text("event", "start")
      .text("name", _scenario->name())
      .count("steps", _scenario->steps().size())
      .num("max_s", _scenario->max_seconds(), 1);
  [self logScenarioStep];
  [self redraw];
}

/// Stops the playing scenario (menu).
- (void)stopScenario {
  if (!_scenario || !_scenario->running()) return;
  _scenario->stop();
  UIApplication.sharedApplication.idleTimerDisabled = NO;
  _log.record("scenario", [self now]).text("event", "stopped").text("name", _scenario->name());
  [_debug showGesture:@"TEST stopped"];
}

/// Logs every step begun since the last call (up to the one playing now) and shows the
/// current one.
- (void)logScenarioStep {
  const Vrst_size now_index = std::min(_scenario->step_index(), _scenario->steps().size() - 1);
  for (; _loggedSteps <= now_index && _loggedSteps < _scenario->steps().size(); ++_loggedSteps)
    _log.record("scenario", [self now])
        .text("event", "step")
        .count("step", _loggedSteps)
        .text("text", _scenario->steps()[_loggedSteps].text);
  if (_scenario->step_index() >= _scenario->steps().size()) return;
  [_debug showGesture:[NSString stringWithFormat:@"TEST %lu/%lu  %s", (unsigned long)now_index + 1,
                                                 (unsigned long)_scenario->steps().size(),
                                                 _scenario->steps()[now_index].text.c_str()]];
}

/// Plays the scenario up to a time and redraws when the camera moved.
/// @param t Seconds since launch.
- (void)playScenarioAt:(double)t {
  Vrst_scenario_frame f;
  f.fb_w = [self fbWidth];
  f.fb_h = [self fbHeight];
  f.px_per_point = [self pxPerPoint];
  f.min_scale = _initialScale * 0.1;  // same limits as the gestures (zoomBy:at:)
  f.max_scale = 2000.0;
  f.tiles_ready = _mapStatus.settled;
  f.world_origin = _mapFrame.center();  // view steps are in degrees
  const Vrst_scenario_tick tick = _scenario->advance(t, _view, f);
  if (tick.jump)
    _view = _mapFrame.start_view(tick.jump->at, tick.jump->mpp, f.px_per_point, f.fb_w, f.fb_h);
  if (tick.step_changed) [self logScenarioStep];
  if (tick.finished) {
    _loggedSteps = _scenario->steps().size();
    UIApplication.sharedApplication.idleTimerDisabled = NO;
    _log.record("scenario", t).text("event", "finished").text("name", _scenario->name());
    [_debug showGesture:@"TEST finished"];
  }
  if (tick.moved) {
    _lastMoveAt = CACurrentMediaTime();
    [self redraw];
  }
}

/// Hides or shows the debug overlay; logging goes on either way.
- (void)toggleOverlay {
  _debug.hidden = !_debug.hidden;
  _log.record("ui", [self now]).text("event", _debug.hidden ? "overlay_hidden" : "overlay_shown");
}

/// Opens the share sheet (AirDrop, Mail, Files ...) with session log files.
/// @param all YES for every stored session, NO for the current one only.
- (void)shareLogs:(BOOL)all {
  _log.record("ui", [self now]).text("event", all ? "share_all" : "share_current");
  _log.flush();
  NSMutableArray<NSURL*>* urls = [NSMutableArray array];
  const std::vector<std::filesystem::path> files = all ? _log.files() : std::vector{_log.path()};
  for (const std::filesystem::path& f : files)
    [urls addObject:[NSURL fileURLWithPath:@(f.string().c_str())]];
  if (urls.count == 0) return;
  UIActivityViewController* share = [[UIActivityViewController alloc] initWithActivityItems:urls
                                                                      applicationActivities:nil];
  share.popoverPresentationController.sourceView = _stats;
  [self presentViewController:share animated:YES completion:nil];
}

/// Deletes every session file except the current one.
- (void)deleteOldLogs {
  _log.delete_old();
  _log.record("ui", [self now]).text("event", "deleted_old_logs");
}

/// Toolbar: place names.
- (void)onNames {
  _showLabels = !_showLabels;
  [_debug showGesture:_showLabels ? @"BUTTON Names on" : @"BUTTON Names off"];
  [self updateToggles];
  [self redraw];
}

// --- GPS position ------------------------------------------------------------------

/// Asks for location permission (when in use) and starts position updates.
- (void)startLocation {
  _location = [[CLLocationManager alloc] init];
  _location.delegate = self;
  _location.desiredAccuracy = kCLLocationAccuracyBest;
  _location.distanceFilter = 2;  // metres; a car moves more than this per update
  [_location requestWhenInUseAuthorization];
}

/// Starts updates once the user allowed location access.
/// @param manager Location manager.
- (void)locationManagerDidChangeAuthorization:(CLLocationManager*)manager {
  const CLAuthorizationStatus s = manager.authorizationStatus;
  if (s == kCLAuthorizationStatusAuthorizedWhenInUse || s == kCLAuthorizationStatusAuthorizedAlways)
    [manager startUpdatingLocation];
}

/// Stores the newest position and redraws the marker.
/// @param manager Location manager.
/// @param locations New positions, oldest first.
- (void)locationManager:(CLLocationManager*)manager didUpdateLocations:(NSArray<CLLocation*>*)locations {
  (void)manager;
  CLLocation* l = locations.lastObject;
  if (!l || l.horizontalAccuracy < 0) return;  // negative accuracy = invalid fix
  _fix = {l.coordinate.latitude, l.coordinate.longitude};
  _fixAccuracy = l.horizontalAccuracy;
  _hasFix = YES;
  [self redraw];
}

/// Location errors (no permission, no signal) keep the last marker; the stats line
/// says "GPS waiting" until a first fix arrives.
/// @param manager Location manager.
/// @param error Error.
- (void)locationManager:(CLLocationManager*)manager didFailWithError:(NSError*)error {
  (void)manager;
  (void)error;
}

// --- gestures ------------------------------------------------------------------

/// Pan, pinch, rotate, double tap and two-finger tap on the map view.
- (void)addGestures {
  UIPanGestureRecognizer* pan = [[UIPanGestureRecognizer alloc] initWithTarget:self action:@selector(onPan:)];
  pan.maximumNumberOfTouches = 1;
  UIPinchGestureRecognizer* pinch = [[UIPinchGestureRecognizer alloc] initWithTarget:self action:@selector(onPinch:)];
  UIRotationGestureRecognizer* rot = [[UIRotationGestureRecognizer alloc] initWithTarget:self
                                                                                  action:@selector(onRotate:)];
  pinch.delegate = self;
  rot.delegate = self;
  UITapGestureRecognizer* dbl = [[UITapGestureRecognizer alloc] initWithTarget:self action:@selector(onDoubleTap:)];
  dbl.numberOfTapsRequired = 2;
  UITapGestureRecognizer* two = [[UITapGestureRecognizer alloc] initWithTarget:self action:@selector(onTwoFingerTap:)];
  two.numberOfTouchesRequired = 2;
  // Lifting two fingers after a quick pinch must not also count as a two-finger tap:
  // that zoomed out 2x and undid the pinch (session log 2026-11-07: all 107 taps fired at
  // the same moment a pinch ended). The tap waits until pinch and twist have failed.
  [two requireGestureRecognizerToFail:pinch];
  [two requireGestureRecognizerToFail:rot];
  _pinchRecognizer = pinch;
  _rotateRecognizer = rot;
  for (UIGestureRecognizer* g in @[ pan, pinch, rot, dbl, two ]) [self.view addGestureRecognizer:g];
  [_debug trackTouchesOf:self.view];
}

/// Lets pinch and rotation run together, like in Apple Maps; no other pair, so a tap
/// cannot fire on top of a pinch.
/// @param a First recognizer.
/// @param b Second recognizer.
/// @return YES only for pinch + rotation.
- (BOOL)gestureRecognizer:(UIGestureRecognizer*)a
    shouldRecognizeSimultaneouslyWithGestureRecognizer:(UIGestureRecognizer*)b {
  return (a == _pinchRecognizer && b == _rotateRecognizer) || (a == _rotateRecognizer && b == _pinchRecognizer);
}

/// Zoom limits: 0.1x the fit scale out, 2000 px per metre in.
/// @param factor Zoom factor to apply.
/// @param p Fixed point in view points.
- (void)zoomBy:(Vrst_float64)factor at:(CGPoint)p {
  const CGFloat s = [self pxPerPoint];
  zoom_at(_view, [self fbWidth], [self fbHeight], p.x * s, p.y * s, factor, _initialScale * 0.1, 2000.0);
}

/// One-finger drag: the map follows the finger.
/// @param g Pan recognizer.
- (void)onPan:(UIPanGestureRecognizer*)g {
  if (!_viewSet) return;
  const CGPoint t = [g translationInView:self.view];
  const CGFloat s = [self pxPerPoint];
  pan_by(_view, t.x * s, t.y * s);
  [g setTranslation:CGPointZero inView:self.view];
  const CGPoint vel = [g velocityInView:self.view];
  [self logGesture:"pan" recognizer:g value:std::hypot(vel.x, vel.y)];
  if (g.state == UIGestureRecognizerStateBegan) [_debug showGesture:@"PAN"];
  if (g.state == UIGestureRecognizerStateBegan && _trackFollow && !_track.empty()) {
    _trackFollow = NO;  // the user looks elsewhere; the menu turns following back on
    _log.record("track", [self now]).text("event", "follow_off");
  }
  if (g.state == UIGestureRecognizerStateEnded)
    [_debug showGesture:[NSString stringWithFormat:@"PAN end %.0f pt/s", std::hypot(vel.x, vel.y)]];
  [self redraw];
}

/// Pinch: zooms around the fingers' centroid and pans when the centroid moves.
/// @param g Pinch recognizer.
- (void)onPinch:(UIPinchGestureRecognizer*)g {
  if (!_viewSet) return;
  const CGPoint c = [g locationInView:self.view];
  if (g.state == UIGestureRecognizerStateBegan) _pinchStartScale = _view.scale;
  if (g.state == UIGestureRecognizerStateBegan || g.numberOfTouches != _lastTouches) {
    _lastCentroid = c;  // a lifted or added finger moves the centroid; do not jump
    _lastTouches = g.numberOfTouches;
  }
  const CGFloat s = [self pxPerPoint];
  pan_by(_view, (c.x - _lastCentroid.x) * s, (c.y - _lastCentroid.y) * s);
  _lastCentroid = c;
  [self zoomBy:g.scale at:c];
  g.scale = 1;
  [self logGesture:"pinch" recognizer:g value:_view.scale / _pinchStartScale];
  NSString* zoom = [NSString stringWithFormat:@"PINCH x%.2f", _view.scale / _pinchStartScale];
  if (g.state == UIGestureRecognizerStateEnded || g.state == UIGestureRecognizerStateCancelled)
    [_debug showGesture:zoom];
  else
    [_debug updateGesture:zoom];
  [self redraw];
}

/// Two-finger twist: rotates the map around the fingers' centroid.
/// @param g Rotation recognizer; UIKit rotation is clockwise on screen.
- (void)onRotate:(UIRotationGestureRecognizer*)g {
  if (!_viewSet) return;
  const CGPoint c = [g locationInView:self.view];
  const CGFloat s = [self pxPerPoint];
  rotate_at(_view, [self fbWidth], [self fbHeight], c.x * s, c.y * s, -g.rotation);
  if (g.state == UIGestureRecognizerStateBegan) _rotationSum = 0;
  _rotationSum += g.rotation;
  g.rotation = 0;
  [self logGesture:"rotate" recognizer:g value:_rotationSum * 180 / kPi];
  // Pinch and twist run together; name the twist only once it is clearly one.
  if (g.state == UIGestureRecognizerStateEnded && std::abs(_rotationSum) > 0.05)
    [_debug showGesture:[NSString stringWithFormat:@"ROTATE %+.0f deg", _rotationSum * 180 / kPi]];
  [self redraw];
}

/// Double tap: zoom in 2x at the tap.
/// @param g Tap recognizer.
- (void)onDoubleTap:(UITapGestureRecognizer*)g {
  if (!_viewSet) return;
  [self logGesture:"double_tap" recognizer:g value:NAN];
  [_debug showGesture:@"DOUBLE TAP -> zoom in 2x"];
  [self zoomBy:2.0 at:[g locationInView:self.view]];
  [self redraw];
}

/// Two-finger tap: zoom out 2x around the fingers.
/// @param g Tap recognizer.
- (void)onTwoFingerTap:(UITapGestureRecognizer*)g {
  if (!_viewSet) return;
  [self logGesture:"two_finger_tap" recognizer:g value:NAN];
  [_debug showGesture:@"2-FINGER TAP -> zoom out 2x"];
  [self zoomBy:0.5 at:[g locationInView:self.view]];
  [self redraw];
}

@end

/// Application delegate: one window; VrstRootController holds the map and the console.
@interface VrstAppDelegate : UIResponder <UIApplicationDelegate>
/// The app's only window.
@property(strong, nonatomic) UIWindow* window;
@end

@implementation VrstAppDelegate

/// Creates the window with the map.
/// @param application The app.
/// @param options Launch options; unused.
/// @return YES.
- (BOOL)application:(UIApplication*)application didFinishLaunchingWithOptions:(NSDictionary*)options {
  (void)application;
  (void)options;
  self.window = [[UIWindow alloc] initWithFrame:UIScreen.mainScreen.bounds];
  VrstDebugConsole* console = [[VrstDebugConsole alloc] initWithFrame:CGRectZero];
  VrstMapController* map = [[VrstMapController alloc] init];
  map.console = console;
  self.window.rootViewController = [[VrstRootController alloc] initWithMap:map console:console];
  [self.window makeKeyAndVisible];
  return YES;
}

@end

/// iOS entry point.
/// @param argc Argument count.
/// @param argv Launch arguments; `--center` and `--mpp` are read by the controller.
/// @return Never returns in practice.
int main(int argc, char* argv[]) {
  @autoreleasepool {
    return UIApplicationMain(argc, argv, nil, NSStringFromClass(VrstAppDelegate.class));
  }
}
