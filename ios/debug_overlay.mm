/// @file
/// VrstDebugOverlay: clock, frame numbers, touch markers and gesture names.
/// Declarations and documentation are in debug_overlay.h.

#import "debug_overlay.h"

#import <QuartzCore/QuartzCore.h>
#import <UIKit/UIGestureRecognizerSubclass.h>

#include <algorithm>
#include <deque>

/// How long a finger trail stays visible, in seconds.
static constexpr double kTrailSeconds = 0.6;
/// How long a lifted finger's marker takes to fade out, in seconds.
static constexpr double kFadeSeconds = 0.5;
/// How long a gesture name stays before it fades, in seconds.
static constexpr double kGestureSeconds = 1.5;
/// Radius of a finger dot in points (about a fingertip).
static constexpr CGFloat kDotRadius = 22;
/// Events kept in the log.
static constexpr NSUInteger kLogLines = 5;

/// One drawn map frame, kept for the per-second numbers.
struct Vrst_frame_sample {
  double at = 0;      ///< CACurrentMediaTime() when the frame was reported.
  double cpu_ms = 0;  ///< CPU time of the draw callback.
};

// --- touch tracker ---------------------------------------------------------------

/// Gesture recognizer that never recognizes: it only forwards every touch to the
/// overlay. It cannot prevent other recognizers and cannot be prevented by them, so it
/// sees each touch from begin to end while pan, pinch and taps work as usual.
@interface VrstTouchTracker : UIGestureRecognizer
/// Overlay that draws the touches.
@property(nonatomic, weak) VrstDebugOverlay* overlay;
@end

/// Overlay methods the tracker calls; implemented below.
@interface VrstDebugOverlay ()
/// Touches started, moved or ended.
/// @param touches Touches of the event.
/// @param ended YES when these touches lifted or were cancelled.
- (void)touchesChanged:(NSSet<UITouch*>*)touches ended:(BOOL)ended;
@end

@implementation VrstTouchTracker

/// Configures the tracker so it never delays or cancels touches of the view.
/// @param view View to watch.
/// @return The tracker.
- (instancetype)initWithView:(UIView*)view {
  self = [super initWithTarget:nil action:nil];
  self.cancelsTouchesInView = NO;
  self.delaysTouchesBegan = NO;
  self.delaysTouchesEnded = NO;
  [view addGestureRecognizer:self];
  return self;
}

/// Never blocks another recognizer.
/// @param other Other recognizer.
/// @return NO.
- (BOOL)canPreventGestureRecognizer:(UIGestureRecognizer*)other {
  (void)other;
  return NO;
}

/// Is never blocked by another recognizer, so it keeps receiving touches.
/// @param other Other recognizer.
/// @return NO.
- (BOOL)canBePreventedByGestureRecognizer:(UIGestureRecognizer*)other {
  (void)other;
  return NO;
}

/// Forwards new touches.
/// @param touches New touches.
/// @param event Their event.
- (void)touchesBegan:(NSSet<UITouch*>*)touches withEvent:(UIEvent*)event {
  (void)event;
  [self.overlay touchesChanged:touches ended:NO];
}

/// Forwards moved touches.
/// @param touches Moved touches.
/// @param event Their event.
- (void)touchesMoved:(NSSet<UITouch*>*)touches withEvent:(UIEvent*)event {
  (void)event;
  [self.overlay touchesChanged:touches ended:NO];
}

/// Forwards lifted touches; fails once all fingers are up, so UIKit resets it.
/// @param touches Lifted touches.
/// @param event Their event.
- (void)touchesEnded:(NSSet<UITouch*>*)touches withEvent:(UIEvent*)event {
  [self.overlay touchesChanged:touches ended:YES];
  [self failWhenAllLifted:event];
}

/// Same as a lift.
/// @param touches Cancelled touches.
/// @param event Their event.
- (void)touchesCancelled:(NSSet<UITouch*>*)touches withEvent:(UIEvent*)event {
  [self.overlay touchesChanged:touches ended:YES];
  [self failWhenAllLifted:event];
}

/// Ends this touch sequence when no finger is down any more.
/// @param event Current event.
- (void)failWhenAllLifted:(UIEvent*)event {
  for (UITouch* t in [event touchesForGestureRecognizer:self])
    if (t.phase != UITouchPhaseEnded && t.phase != UITouchPhaseCancelled) return;
  self.state = UIGestureRecognizerStateFailed;
}

@end

// --- finger marker -----------------------------------------------------------------

/// Dot and trail of one finger.
@interface VrstTouchMark : NSObject
/// Finger number shown in the dot (1 = first finger down).
@property(nonatomic) NSUInteger finger;
/// Filled circle under the finger.
@property(nonatomic, strong) CAShapeLayer* dot;
/// Path of the last kTrailSeconds of movement.
@property(nonatomic, strong) CAShapeLayer* trail;
@end

@implementation VrstTouchMark {
  /// Recent positions, oldest first.
  std::deque<CGPoint> _points;
  /// Times of _points (CACurrentMediaTime()).
  std::deque<double> _times;
}

/// Adds a position and redraws dot and trail.
/// @param p Finger position in overlay points.
/// @param now Current time.
- (void)moveTo:(CGPoint)p at:(double)now {
  _points.push_back(p);
  _times.push_back(now);
  self.dot.position = p;
  [self pruneAt:now];
}

/// Drops trail points older than kTrailSeconds and rebuilds the trail path.
/// @param now Current time.
- (void)pruneAt:(double)now {
  while (_times.size() > 1 && now - _times.front() > kTrailSeconds) {
    _times.pop_front();
    _points.pop_front();
  }
  UIBezierPath* path = [UIBezierPath bezierPath];
  for (std::size_t i = 0; i < _points.size(); ++i) {
    if (i == 0)
      [path moveToPoint:_points[i]];
    else
      [path addLineToPoint:_points[i]];
  }
  self.trail.path = path.CGPath;
}

@end

// --- overlay -------------------------------------------------------------------------

@implementation VrstDebugOverlay {
  /// Time of day with tenths and seconds since launch.
  UILabel* _clock;
  /// Frame and tile numbers.
  UILabel* _perf;
  /// Last kLogLines events with their times.
  UILabel* _log;
  /// Current gesture in large letters.
  UILabel* _gesture;
  /// Formats _clock.
  NSDateFormatter* _timeFormat;
  /// CACurrentMediaTime() of the previous display-link tick, for the gap.
  CFTimeInterval _lastTick;
  /// Last time the gesture name changed; it fades kGestureSeconds later.
  double _gestureAt;
  /// Clock text shown, to skip identical updates.
  NSString* _clockText;
  /// Frames of the last second.
  std::deque<Vrst_frame_sample> _frames;
  NSUInteger _visible;   ///< Tiles on screen in the last frame.
  NSUInteger _missing;   ///< Tiles on screen not built yet in the last frame.
  NSUInteger _building;  ///< Tiles queued or being built after the last frame.
  /// Bytes uploaded in the frames of the last second.
  std::deque<NSUInteger> _uploads;
  /// Log lines, newest last.
  NSMutableArray<NSString*>* _events;
  /// Marker of every finger that is down.
  NSMapTable<UITouch*, VrstTouchMark*>* _marks;
  /// Drives the clock, trail pruning and the idle state of the numbers.
  CADisplayLink* _tick;
}

/// Builds the labels and starts the clock.
/// @param frame Initial frame; the controller pins the overlay to the map view.
/// @return The overlay.
- (instancetype)initWithFrame:(CGRect)frame {
  self = [super initWithFrame:frame];
  self.userInteractionEnabled = NO;  // never takes touches; VrstTouchTracker watches them
  self.backgroundColor = UIColor.clearColor;
  _startTime = CACurrentMediaTime();
  _events = [NSMutableArray array];
  _marks = [NSMapTable weakToStrongObjectsMapTable];
  _timeFormat = [[NSDateFormatter alloc] init];
  _timeFormat.dateFormat = @"HH:mm:ss.S";

  _clock = [self labelWithSize:20 weight:UIFontWeightBold];
  _perf = [self labelWithSize:11 weight:UIFontWeightRegular];
  _log = [self labelWithSize:11 weight:UIFontWeightRegular];
  _log.textColor = [UIColor colorWithWhite:0.85 alpha:1];
  UIView* rule = [[UIView alloc] init];  // separates the numbers from the event log
  rule.backgroundColor = [UIColor colorWithWhite:0.4 alpha:1];
  rule.translatesAutoresizingMaskIntoConstraints = NO;
  [rule.heightAnchor constraintEqualToConstant:0.5].active = YES;
  UIStackView* panel = [[UIStackView alloc] initWithArrangedSubviews:@[ _clock, _perf, rule, _log ]];
  panel.axis = UILayoutConstraintAxisVertical;
  panel.alignment = UIStackViewAlignmentFill;
  panel.spacing = 2;
  panel.layoutMargins = UIEdgeInsetsMake(4, 6, 4, 6);
  panel.layoutMarginsRelativeArrangement = YES;
  panel.backgroundColor = [UIColor colorWithWhite:0 alpha:0.55];
  panel.layer.cornerRadius = 6;
  panel.translatesAutoresizingMaskIntoConstraints = NO;
  [self addSubview:panel];
  _panel = panel;

  _gesture = [self labelWithSize:24 weight:UIFontWeightHeavy];
  _gesture.textAlignment = NSTextAlignmentCenter;
  _gesture.backgroundColor = [UIColor colorWithRed:0.85 green:0.1 blue:0.45 alpha:0.8];
  _gesture.layer.cornerRadius = 10;
  _gesture.layer.masksToBounds = YES;
  _gesture.alpha = 0;
  [self addSubview:_gesture];
  UILayoutGuide* safe = self.safeAreaLayoutGuide;
  [NSLayoutConstraint activateConstraints:@[
    [_gesture.centerXAnchor constraintEqualToAnchor:self.centerXAnchor],
    [_gesture.bottomAnchor constraintEqualToAnchor:safe.bottomAnchor constant:-64],  // above the toolbar
    [_gesture.widthAnchor constraintLessThanOrEqualToAnchor:self.widthAnchor constant:-24],
  ]];

  _tick = [CADisplayLink displayLinkWithTarget:self selector:@selector(onTick)];
  [_tick addToRunLoop:NSRunLoop.mainRunLoop forMode:NSRunLoopCommonModes];
  [self updatePerf];
  return self;
}

/// Stops the display link, which holds a strong reference to the overlay.
- (void)removeFromSuperview {
  [_tick invalidate];
  [super removeFromSuperview];
}

/// A white monospaced label for the overlay.
/// @param size Font size in points.
/// @param weight Font weight.
/// @return The label, using Auto Layout.
- (UILabel*)labelWithSize:(CGFloat)size weight:(UIFontWeight)weight {
  UILabel* l = [[UILabel alloc] init];
  l.translatesAutoresizingMaskIntoConstraints = NO;
  l.numberOfLines = 0;
  l.font = [UIFont monospacedSystemFontOfSize:size weight:weight];
  l.textColor = UIColor.whiteColor;
  return l;
}

/// Seconds since the overlay started.
/// @return Seconds.
- (double)elapsed {
  return CACurrentMediaTime() - _startTime;
}

- (void)trackTouchesOf:(UIView*)view {
  VrstTouchTracker* t = [[VrstTouchTracker alloc] initWithView:view];
  t.overlay = self;
}

- (void)showGesture:(NSString*)text {
  [self updateGesture:text];
  [_events addObject:[NSString stringWithFormat:@"%7.1f  %@", [self elapsed], text]];
  while (_events.count > kLogLines) [_events removeObjectAtIndex:0];
  _log.text = [_events componentsJoinedByString:@"\n"];
}

- (void)updateGesture:(NSString*)text {
  _gesture.text = [NSString stringWithFormat:@"  %@  ", text];
  _gesture.alpha = 1;
  _gestureAt = CACurrentMediaTime();
}

- (void)frameDrawnInMs:(double)cpuMs
               visible:(NSUInteger)visible
               missing:(NSUInteger)missing
              building:(NSUInteger)building
           uploadBytes:(NSUInteger)uploadBytes {
  _frames.push_back({CACurrentMediaTime(), cpuMs});
  _uploads.push_back(uploadBytes);
  _visible = visible;
  _missing = missing;
  _building = building;
  [self updatePerf];
}

/// Rewrites the frame numbers from the frames of the last second. The frame interval
/// counts only gaps under 0.25 s: longer ones mean the map was idle, not slow.
- (void)updatePerf {
  const double now = CACurrentMediaTime();
  while (!_frames.empty() && now - _frames.front().at > 1.0) {
    _frames.pop_front();
    _uploads.pop_front();
  }
  if (_frames.empty()) {
    _perf.text = [NSString stringWithFormat:@"idle (no map frames)\ntiles %lu vis  %lu miss  %lu build",
                                            (unsigned long)_visible, (unsigned long)_missing,
                                            (unsigned long)_building];
    return;
  }
  double cpu_sum = 0, cpu_max = 0, gap_sum = 0, gap_max = 0;
  int gaps = 0;
  for (std::size_t i = 0; i < _frames.size(); ++i) {
    cpu_sum += _frames[i].cpu_ms;
    cpu_max = std::max(cpu_max, _frames[i].cpu_ms);
    if (i == 0) continue;
    const double gap = (_frames[i].at - _frames[i - 1].at) * 1000;
    if (gap >= 250) continue;
    gap_sum += gap;
    gap_max = std::max(gap_max, gap);
    ++gaps;
  }
  NSUInteger up = 0;
  for (const NSUInteger b : _uploads) up += b;
  const double n = static_cast<double>(_frames.size());
  _perf.text = [NSString
      stringWithFormat:@"fps %2lu  frame %4.1f / %4.1f ms\ndraw %4.1f / %4.1f ms (cpu)\n"
                       @"tiles %lu vis  %lu miss  %lu build\nupload %.1f MB/s",
                       (unsigned long)_frames.size(), gaps > 0 ? gap_sum / gaps : 0.0, gap_max, cpu_sum / n, cpu_max,
                       (unsigned long)_visible, (unsigned long)_missing, (unsigned long)_building,
                       static_cast<double>(up) / 1048576.0];
}

/// Display link: clock, trail pruning, gesture fade and idle numbers.
- (void)onTick {
  const double now = CACurrentMediaTime();
  if (self.tickHandler) self.tickHandler(now, _lastTick > 0 ? (now - _lastTick) * 1000 : 0);
  _lastTick = now;
  if (self.hidden) return;
  NSString* clock = [NSString stringWithFormat:@"%@  t %.1f s", [_timeFormat stringFromDate:NSDate.date],
                                               now - _startTime];
  if (![clock isEqualToString:_clockText]) {
    _clockText = clock;
    _clock.text = clock;
  }
  [CATransaction begin];
  [CATransaction setDisableActions:YES];
  for (VrstTouchMark* m in _marks.objectEnumerator) [m pruneAt:now];
  [CATransaction commit];
  if (_gesture.alpha > 0 && now - _gestureAt > kGestureSeconds) {
    [UIView animateWithDuration:0.4 animations:^{ self->_gesture.alpha = 0; }];
    _gestureAt = now + 1e9;  // fade once
  }
  if (!_frames.empty() && now - _frames.back().at > 1.0) [self updatePerf];
}

/// Colour of a finger: yellow for the first, cyan for the second, then magenta.
/// @param index How many fingers were already down.
/// @return Colour.
- (UIColor*)colorForFinger:(NSUInteger)index {
  if (index == 0) return UIColor.systemYellowColor;
  if (index == 1) return UIColor.systemTealColor;
  return UIColor.systemPinkColor;
}

- (void)touchesChanged:(NSSet<UITouch*>*)touches ended:(BOOL)ended {
  const double now = CACurrentMediaTime();
  [CATransaction begin];
  [CATransaction setDisableActions:YES];
  for (UITouch* t in touches) {
    const CGPoint p = [t locationInView:self];
    VrstTouchMark* m = [_marks objectForKey:t];
    if (!m) {
      m = [self newMarkWithColor:[self colorForFinger:_marks.count] number:_marks.count + 1];
      m.finger = _marks.count + 1;
      [_marks setObject:m forKey:t];
    }
    if (self.touchHandler) self.touchHandler(t, m.finger, ended);
    [m moveTo:p at:now];
    if (ended) {
      [self fadeOut:m];
      [_marks removeObjectForKey:t];
    }
  }
  [CATransaction commit];
}

/// Creates the layers of a new finger marker.
/// @param color Finger colour.
/// @param number Finger number shown in the dot (1 = first finger down).
/// @return The marker, already on screen.
- (VrstTouchMark*)newMarkWithColor:(UIColor*)color number:(NSUInteger)number {
  VrstTouchMark* m = [[VrstTouchMark alloc] init];
  m.trail = [CAShapeLayer layer];
  m.trail.strokeColor = [color colorWithAlphaComponent:0.8].CGColor;
  m.trail.fillColor = nil;
  m.trail.lineWidth = 4;
  m.trail.lineCap = kCALineCapRound;
  m.trail.lineJoin = kCALineJoinRound;
  m.dot = [CAShapeLayer layer];
  m.dot.bounds = CGRectMake(0, 0, 2 * kDotRadius, 2 * kDotRadius);
  m.dot.path = [UIBezierPath bezierPathWithOvalInRect:m.dot.bounds].CGPath;
  m.dot.fillColor = [color colorWithAlphaComponent:0.35].CGColor;
  m.dot.strokeColor = color.CGColor;
  m.dot.lineWidth = 3;
  CATextLayer* label = [CATextLayer layer];
  label.string = [NSString stringWithFormat:@"%lu", (unsigned long)number];
  label.font = (__bridge CFTypeRef)[UIFont boldSystemFontOfSize:14];
  label.fontSize = 14;
  label.foregroundColor = UIColor.blackColor.CGColor;
  label.alignmentMode = kCAAlignmentCenter;
  label.contentsScale = UIScreen.mainScreen.scale;
  label.frame = CGRectMake(0, kDotRadius - 9, 2 * kDotRadius, 18);
  [m.dot addSublayer:label];
  [self.layer addSublayer:m.trail];
  [self.layer addSublayer:m.dot];
  return m;
}

/// Fades a lifted finger's dot and trail out, then removes them.
/// @param m Marker of the lifted finger.
- (void)fadeOut:(VrstTouchMark*)m {
  for (CALayer* l in @[ m.dot, m.trail ]) {
    CABasicAnimation* a = [CABasicAnimation animationWithKeyPath:@"opacity"];
    a.fromValue = @1.0;
    a.toValue = @0.0;
    a.duration = kFadeSeconds;
    l.opacity = 0;
    [l addAnimation:a forKey:@"fade"];
  }
  dispatch_after(dispatch_time(DISPATCH_TIME_NOW, static_cast<int64_t>(kFadeSeconds * NSEC_PER_SEC)),
                 dispatch_get_main_queue(), ^{
                   [m.dot removeFromSuperlayer];
                   [m.trail removeFromSuperlayer];
                 });
}

@end
