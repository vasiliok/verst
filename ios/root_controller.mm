/// @file
/// VrstRootController: map and console side by side or one over the other.
/// Declarations and documentation are in root_controller.h.

#import "root_controller.h"

/// Part of the screen height the console takes in Split.
static constexpr CGFloat kConsoleSplit = 0.45;
/// Layout change animation, seconds.
static constexpr NSTimeInterval kLayoutSeconds = 0.2;

@implementation VrstRootController {
  /// Map controller, a child.
  UIViewController* _map;
  /// Debug console.
  VrstDebugConsole* _console;
  /// Console height 0 (Closed).
  NSLayoutConstraint* _consoleClosed;
  /// Console height kConsoleSplit of the screen (Split).
  NSLayoutConstraint* _consoleSplit;
  /// Console height of the whole screen (Full).
  NSLayoutConstraint* _consoleFull;
  /// Map top at the screen top (Closed, Full).
  NSLayoutConstraint* _mapAtTop;
  /// Map top at the console's bottom (Split).
  NSLayoutConstraint* _mapBelowConsole;
}

- (instancetype)initWithMap:(UIViewController*)map console:(VrstDebugConsole*)console {
  self = [super initWithNibName:nil bundle:nil];
  _map = map;
  _console = console;
  return self;
}

/// Adds the map as a child and the console over it, and pins both for Closed.
- (void)viewDidLoad {
  [super viewDidLoad];
  self.view.backgroundColor = UIColor.blackColor;
  [self addChildViewController:_map];
  UIView* map = _map.view;
  map.translatesAutoresizingMaskIntoConstraints = NO;
  [self.view addSubview:map];
  [_map didMoveToParentViewController:self];
  _console.translatesAutoresizingMaskIntoConstraints = NO;
  [self.view addSubview:_console];

  UIView* v = self.view;
  _consoleClosed = [_console.heightAnchor constraintEqualToConstant:0];
  _consoleSplit = [_console.heightAnchor constraintEqualToAnchor:v.heightAnchor multiplier:kConsoleSplit];
  _consoleFull = [_console.heightAnchor constraintEqualToAnchor:v.heightAnchor];
  _mapAtTop = [map.topAnchor constraintEqualToAnchor:v.topAnchor];
  _mapBelowConsole = [map.topAnchor constraintEqualToAnchor:_console.bottomAnchor];
  [NSLayoutConstraint activateConstraints:@[
    [_console.topAnchor constraintEqualToAnchor:v.topAnchor],
    [_console.leadingAnchor constraintEqualToAnchor:v.leadingAnchor],
    [_console.trailingAnchor constraintEqualToAnchor:v.trailingAnchor],
    [map.leadingAnchor constraintEqualToAnchor:v.leadingAnchor],
    [map.trailingAnchor constraintEqualToAnchor:v.trailingAnchor],
    [map.bottomAnchor constraintEqualToAnchor:v.bottomAnchor],
    _consoleClosed,
    _mapAtTop,
  ]];
  __weak VrstRootController* weak = self;
  _console.layoutHandler = ^(VrstConsoleLayout layout) { [weak place:layout]; };
}

/// Moves the console and the map to a layout, animated. The console is shown before
/// it grows and hidden after it shrank to nothing.
/// @param layout New layout.
- (void)place:(VrstConsoleLayout)layout {
  [NSLayoutConstraint deactivateConstraints:@[ _consoleClosed, _consoleSplit, _consoleFull, _mapAtTop, _mapBelowConsole ]];
  switch (layout) {
    case VrstConsoleLayoutClosed:
      [NSLayoutConstraint activateConstraints:@[ _consoleClosed, _mapAtTop ]];
      break;
    case VrstConsoleLayoutSplit:
      [NSLayoutConstraint activateConstraints:@[ _consoleSplit, _mapBelowConsole ]];
      break;
    case VrstConsoleLayoutFull:
      [NSLayoutConstraint activateConstraints:@[ _consoleFull, _mapAtTop ]];
      break;
  }
  if (layout != VrstConsoleLayoutClosed) _console.hidden = NO;
  [UIView animateWithDuration:kLayoutSeconds
      animations:^{ [self.view layoutIfNeeded]; }
      completion:^(BOOL) {
        if (self->_console.layout == VrstConsoleLayoutClosed) self->_console.hidden = YES;
      }];
}

/// The map decides the status bar style (light text over the dark map).
/// @return The map controller.
- (UIViewController*)childViewControllerForStatusBarStyle {
  return _map;
}

@end
