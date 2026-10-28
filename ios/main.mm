/// @file
/// verst iPhone app: a full-screen map drawn with OpenGL ES 3.0.
///
/// A thin UIKit shell around the shared C++ core: Vrst_nav_data maps the bundled nav
/// files, Vrst_map_mesh builds the geometry and Vrst_map_renderer draws it, exactly
/// as in the desktop viewer. This file only adds the GL context (GLKView), touch
/// gestures, a stats line and a toolbar.
///
/// Gestures: one-finger drag = pan, pinch = zoom (and pan with the two fingers),
/// two-finger twist = rotate, double tap = zoom in, two-finger tap = zoom out.
/// Launch arguments `--center <lat> <lon>` and `--mpp <metres>` set the start view,
/// like navview.

#include "gl_api.h"  // OpenGL ES 3.0; before GLKit so the ES3 header wins

#import <GLKit/GLKit.h>
#import <UIKit/UIKit.h>

#include <cmath>
#include <exception>

#include "map_renderer.h"
#include "map_scene.h"
#include "nav_files.h"

using namespace nav;

/// Full-screen map controller. Owns the shared C++ map state and turns UIKit touch
/// gestures into camera changes; draws only when something changed.
@interface VrstMapController : GLKViewController <UIGestureRecognizerDelegate>
@end

@implementation VrstMapController {
  /// Mapped nav files from the app bundle; spans into them stay valid while open.
  Vrst_nav_data _data;
  /// GPU-ready geometry built from _data.
  Vrst_map_mesh _mesh;
  /// Shared OpenGL renderer; initialised once the context exists.
  Vrst_map_renderer _renderer;
  /// Current camera.
  Vrst_view _view;
  /// Display switches changed by the toolbar.
  Vrst_draw_options _opt;
  /// Start view center: kDefaultPoint or --center.
  Vrst_coord _startCenter;
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
  UIBarButtonItem* _roadsItem;  ///< Road class menu button.
  /// Load error shown instead of the map, or nil.
  NSString* _error;
}

/// Uses a GLKView as the root view.
- (void)loadView {
  self.view = [[GLKView alloc] initWithFrame:UIScreen.mainScreen.bounds];
}

/// Creates the GL context, loads the map and builds the overlay and gestures.
- (void)viewDidLoad {
  [super viewDidLoad];
  _startCenter = kDefaultPoint;
  _startMpp = kDefaultMetresPerPoint;
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
    NSString* dir = [NSBundle.mainBundle.resourcePath stringByAppendingPathComponent:@"map"];
    _data.open(dir.UTF8String);
    _mesh.build(_data);
    _loaded = YES;
    _renderer.init(_mesh);
    _glReady = YES;
  } catch (const std::exception& e) {
    _error = [NSString stringWithFormat:@"cannot load map: %s", e.what()];
  }
  [self buildOverlay];
  [self addGestures];
}

/// Releases the GL objects while the context is still alive.
- (void)dealloc {
  if (_glReady) {
    [EAGLContext setCurrentContext:_context];
    _renderer.destroy();
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
        _startCenter = {Vrst_int32(std::lround(lat * 1e7)), Vrst_int32(std::lround(lon * 1e7))};
      i += 2;
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
  [self.view setNeedsDisplay];
}

/// Redraws after rotation or any other size change.
- (void)viewDidLayoutSubviews {
  [super viewDidLayoutSubviews];
  [self redraw];
}

/// GLKView draw callback: sets the start view on the first frame, then draws.
/// @param view The GLKView, with its framebuffer bound.
/// @param rect Dirty rectangle; the whole map is always redrawn.
- (void)glkView:(GLKView*)view drawInRect:(CGRect)rect {
  (void)rect;
  if (!_glReady) {
    glClearColor(kBackground[0], kBackground[1], kBackground[2], 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    return;
  }
  const int w = int(view.drawableWidth), h = int(view.drawableHeight);
  if (!_viewSet && w > 0 && h > 0) {
    _initialScale = _mesh.fit_scale(w, h);
    [self resetView];
    _viewSet = YES;
  }
  _opt.point_px = Vrst_float32(2.5 * [self pxPerPoint]);
  _renderer.draw(_view, w, h, _opt);
  [self updateStats];
}

/// Start view: _startCenter at _startMpp, or the whole map if it is outside the data.
- (void)resetView {
  _view = _mesh.start_view(_data.graph().bbox(), _startCenter, _startMpp, [self pxPerPoint], [self fbWidth],
                           [self fbHeight]);
}

/// Whole map, no rotation.
- (void)fitView {
  _view = {{0, 0}, _mesh.fit_scale([self fbWidth], [self fbHeight]), 0};
}

// --- overlay -------------------------------------------------------------------

/// Stats label at the top and toolbar at the bottom, both inside the safe area.
- (void)buildOverlay {
  _stats = [[UILabel alloc] init];
  _stats.translatesAutoresizingMaskIntoConstraints = NO;
  _stats.numberOfLines = 0;
  _stats.font = [UIFont monospacedSystemFontOfSize:11 weight:UIFontWeightRegular];
  _stats.textColor = UIColor.whiteColor;
  _stats.backgroundColor = [UIColor colorWithWhite:0 alpha:0.45];
  _stats.layer.cornerRadius = 6;
  _stats.layer.masksToBounds = YES;
  [self.view addSubview:_stats];

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
  UIBarButtonItem* flex = [[UIBarButtonItem alloc] initWithBarButtonSystemItem:UIBarButtonSystemItemFlexibleSpace
                                                                        target:nil action:nil];
  bar.items = @[ fit, flex, start, flex, _roadsItem, flex, _onewayItem, flex, _dotsItem ];
  [self.view addSubview:bar];

  UILayoutGuide* safe = self.view.safeAreaLayoutGuide;
  [NSLayoutConstraint activateConstraints:@[
    [_stats.topAnchor constraintEqualToAnchor:safe.topAnchor constant:6],
    [_stats.leadingAnchor constraintEqualToAnchor:safe.leadingAnchor constant:8],
    [bar.leadingAnchor constraintEqualToAnchor:self.view.leadingAnchor],
    [bar.trailingAnchor constraintEqualToAnchor:self.view.trailingAnchor],
    [bar.bottomAnchor constraintEqualToAnchor:safe.bottomAnchor],
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
    UIAction* a = [UIAction actionWithTitle:@(kGroupNames[i]) image:nil identifier:nil
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
}

/// Refreshes the stats line: counts, zoom and ground resolution, or the load error.
- (void)updateStats {
  if (_error) {
    _stats.text = [NSString stringWithFormat:@" %@ ", _error];
    return;
  }
  if (!_viewSet) return;
  const Vrst_float64 lat = inverse_mercator(_mesh.center()).x * kPi / 180;
  _stats.text = [NSString stringWithFormat:@" V %u  S %u  A %u  P %zu \n zoom %.2fx  %.2f m/pt  dead ends %u ",
                 _data.graph().num_vertices(), _data.edges().num_segments(), _data.graph().num_arcs(),
                 _data.geom().points().size(), _view.scale / _initialScale,
                 [self pxPerPoint] * std::cos(lat) / _view.scale, _mesh.dead_ends()];
}

/// Toolbar: whole map.
- (void)onFit {
  [self fitView];
  [self redraw];
}

/// Toolbar: start view.
- (void)onStart {
  [self resetView];
  [self redraw];
}

/// Toolbar: oneway highlight.
- (void)onOneway {
  _opt.oneway = !_opt.oneway;
  [self updateToggles];
  [self redraw];
}

/// Toolbar: vertex dots.
- (void)onDots {
  _opt.show_vertices = !_opt.show_vertices;
  [self updateToggles];
  [self redraw];
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
  for (UIGestureRecognizer* g in @[ pan, pinch, rot, dbl, two ]) [self.view addGestureRecognizer:g];
}

/// Lets pinch and rotation run together, like in Apple Maps.
/// @param a First recognizer.
/// @param b Second recognizer.
/// @return YES for any pair.
- (BOOL)gestureRecognizer:(UIGestureRecognizer*)a
    shouldRecognizeSimultaneouslyWithGestureRecognizer:(UIGestureRecognizer*)b {
  (void)a;
  (void)b;
  return YES;
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
  [self redraw];
}

/// Pinch: zooms around the fingers' centroid and pans when the centroid moves.
/// @param g Pinch recognizer.
- (void)onPinch:(UIPinchGestureRecognizer*)g {
  if (!_viewSet) return;
  const CGPoint c = [g locationInView:self.view];
  if (g.state == UIGestureRecognizerStateBegan || g.numberOfTouches != _lastTouches) {
    _lastCentroid = c;  // a lifted or added finger moves the centroid; do not jump
    _lastTouches = g.numberOfTouches;
  }
  const CGFloat s = [self pxPerPoint];
  pan_by(_view, (c.x - _lastCentroid.x) * s, (c.y - _lastCentroid.y) * s);
  _lastCentroid = c;
  [self zoomBy:g.scale at:c];
  g.scale = 1;
  [self redraw];
}

/// Two-finger twist: rotates the map around the fingers' centroid.
/// @param g Rotation recognizer; UIKit rotation is clockwise on screen.
- (void)onRotate:(UIRotationGestureRecognizer*)g {
  if (!_viewSet) return;
  const CGPoint c = [g locationInView:self.view];
  const CGFloat s = [self pxPerPoint];
  rotate_at(_view, [self fbWidth], [self fbHeight], c.x * s, c.y * s, -g.rotation);
  g.rotation = 0;
  [self redraw];
}

/// Double tap: zoom in 2x at the tap.
/// @param g Tap recognizer.
- (void)onDoubleTap:(UITapGestureRecognizer*)g {
  if (!_viewSet) return;
  [self zoomBy:2.0 at:[g locationInView:self.view]];
  [self redraw];
}

/// Two-finger tap: zoom out 2x around the fingers.
/// @param g Tap recognizer.
- (void)onTwoFingerTap:(UITapGestureRecognizer*)g {
  if (!_viewSet) return;
  [self zoomBy:0.5 at:[g locationInView:self.view]];
  [self redraw];
}

@end

/// Application delegate: one window with the map controller.
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
  self.window.rootViewController = [[VrstMapController alloc] init];
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
