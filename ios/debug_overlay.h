/// @file
/// VrstDebugOverlay: what a screen recording of the phone needs to be understood
/// afterwards, drawn on top of the map:
/// - a clock (time of day with tenths, and seconds since launch) to point at moments;
/// - frame numbers: draws per second, frame interval, CPU draw time, tile counts, uploads;
/// - a dot under every finger with a short trail, fading out when the finger lifts;
/// - the gesture being made in large letters (PAN, PINCH ×1.8, DOUBLE TAP …) and a log
///   of the last events with their times.
///
/// It never takes touches: a recognizer that never fires watches them, so the map
/// gestures behave exactly as without the overlay.

#import <UIKit/UIKit.h>

/// Debug layer over the map view for screen recordings; see the file comment.
@interface VrstDebugOverlay : UIView

/// Clock, frame numbers and event log; the map controller places it under its stats.
@property(nonatomic, readonly) UIView* panel;

/// CACurrentMediaTime() of the run start: the clock shows `t` relative to it. The map
/// controller sets it, so the clock and the session log use the same `t`.
@property(nonatomic) CFTimeInterval startTime;

/// Called for every touch change, also while the overlay is hidden (session log).
/// Arguments: the touch, its finger number (1 = first finger down), YES when it lifted.
@property(nonatomic, copy) void (^touchHandler)(UITouch* touch, NSUInteger finger, BOOL ended);

/// Called on every display-link tick, also while hidden. Arguments: CACurrentMediaTime()
/// of the tick and the gap since the previous tick in ms (a long gap = blocked main thread).
@property(nonatomic, copy) void (^tickHandler)(CFTimeInterval now, double gapMs);

/// Starts following the touches of a view without taking them from its gestures.
/// @param view View whose touches are shown; the overlay must cover it exactly.
- (void)trackTouchesOf:(UIView*)view;

/// Shows a finished gesture or a button press in large letters and logs it.
/// @param text Short description, e.g. "DOUBLE TAP → zoom in 2×".
- (void)showGesture:(NSString*)text;

/// Shows a running gesture (pinch factor so far, …) without logging it.
/// @param text Short description, e.g. "PINCH ×1.42".
- (void)updateGesture:(NSString*)text;

/// Reports one drawn map frame.
/// @param cpuMs CPU time of the draw callback in milliseconds.
/// @param visible Tiles on screen.
/// @param missing Tiles on screen not built yet.
/// @param building Tiles queued or being built by the loader.
/// @param uploadBytes Bytes of tile data uploaded to the GPU in this frame.
- (void)frameDrawnInMs:(double)cpuMs
               visible:(NSUInteger)visible
               missing:(NSUInteger)missing
              building:(NSUInteger)building
           uploadBytes:(NSUInteger)uploadBytes;

@end
