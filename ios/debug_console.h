/// @file
/// VrstDebugConsole: a debug console in the style of a game console:
/// - live status lines at the top (the map provider's work, set by the controller), in
///   Full only: in Split the map's own stats box shows them;
/// - the session log below, newest at the bottom, following new lines while the view is
///   scrolled to the end and holding still while the user reads older lines;
/// - filter buttons by record type (e.g. map work, user input) and a text filter that
///   keeps only lines containing its text;
/// - Split / Full: shares the screen with the map (both usable) or covers it.
///
/// The console does not place itself: VrstRootController lays out the console and the
/// map for the current layout. The controller feeds it; the console does not know the
/// log or the map.

#import <UIKit/UIKit.h>

/// How the console uses the screen.
typedef NS_ENUM(NSInteger, VrstConsoleLayout) {
  VrstConsoleLayoutClosed,  ///< Hidden; the map has the whole screen.
  VrstConsoleLayoutSplit,   ///< Upper part of the screen; the map below stays usable.
  VrstConsoleLayoutFull,    ///< Whole screen, over the map.
};

/// One filter button: its title and the record types it shows.
@interface VrstConsoleFilter : NSObject
/// Button title.
@property(nonatomic, readonly, copy) NSString* title;
/// Shown types, or nil when `hidden` is used instead.
@property(nonatomic, readonly, copy) NSSet<NSString*>* only;
/// Types not shown; nil or empty with `only` nil = every type.
@property(nonatomic, readonly, copy) NSSet<NSString*>* hidden;

/// A filter that shows only some record types.
/// @param title Button title.
/// @param types Record types shown.
/// @return Filter.
+ (instancetype)filterNamed:(NSString*)title only:(NSSet<NSString*>*)types;

/// A filter that shows every record type except some (so new types show up too).
/// @param title Button title.
/// @param types Record types not shown; empty = all.
/// @return Filter.
+ (instancetype)filterNamed:(NSString*)title except:(NSSet<NSString*>*)types;
@end

/// Debug console; see the file comment.
@interface VrstDebugConsole : UIView

/// Filter buttons, left to right; the first is selected at start.
@property(nonatomic, copy) NSArray<VrstConsoleFilter*>* filters;

/// Current layout; change it with show: or close.
@property(nonatomic, readonly) VrstConsoleLayout layout;

/// Called on every layout change, before any animation (VrstRootController moves the
/// views). Argument: the new layout.
@property(nonatomic, copy) void (^layoutHandler)(VrstConsoleLayout layout);

/// Called after the console was closed by its Close button.
@property(nonatomic, copy) void (^closeHandler)(void);

/// YES while the layout is not Closed.
@property(nonatomic, readonly) BOOL shown;

/// Opens the console or changes its layout.
/// @param layout Split or Full.
- (void)show:(VrstConsoleLayout)layout;

/// Closes the console; the map gets the whole screen again.
- (void)close;

/// Replaces the status lines at the top.
/// @param lines Lines, top to bottom.
- (void)setStatusLines:(NSArray<NSString*>*)lines;

/// Removes all log lines (before the controller feeds the log again from its start).
- (void)clearLog;

/// Adds log lines at the bottom and redraws the log when any of them is shown.
/// @param lines Readable log lines, `<t> <type> key=value ...`, oldest first.
- (void)appendLogLines:(NSArray<NSString*>*)lines;

@end
