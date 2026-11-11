/// @file
/// VrstRootController: the window's root. It holds the map controller as a child and
/// the debug console, and places them for the console's layout:
/// - Closed: the map has the whole screen;
/// - Split: the console on top (kConsoleSplit of the height), the map below it, resized,
///   so the user pans and zooms while watching the log;
/// - Full: the console over the whole screen.

#import <UIKit/UIKit.h>

#import "debug_console.h"

/// Root controller; see the file comment.
@interface VrstRootController : UIViewController

/// Creates the root with its two parts.
/// @param map Map controller; becomes a child, its view is resized in Split.
/// @param console Console; the root sets its layoutHandler.
/// @return The controller.
- (instancetype)initWithMap:(UIViewController*)map console:(VrstDebugConsole*)console;

@end
