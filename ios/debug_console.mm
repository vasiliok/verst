/// @file
/// VrstDebugConsole: status and session log beside or over the map.
/// Declarations and documentation are in debug_console.h.

#import "debug_console.h"

#include <algorithm>
#include <deque>

/// Log lines the console keeps.
static constexpr std::size_t kKeptLines = 2000;
/// Lines shown after filtering; more makes the text view slow to lay out.
static constexpr std::size_t kShownLines = 600;
/// Distance from the end, in points, that still counts as "at the end" (follow new lines).
static constexpr CGFloat kFollowSlack = 24;
/// Indent of the wrapped part of a long line, points, so it does not look like a new record.
static constexpr CGFloat kWrapIndent = 18;

/// One kept log line.
struct Vrst_console_line {
  NSString* text = nil;  ///< Readable line.
  NSString* type = nil;  ///< Record type: the second word of the line.
};

@implementation VrstConsoleFilter

+ (instancetype)filterNamed:(NSString*)title only:(NSSet<NSString*>*)types {
  VrstConsoleFilter* f = [[VrstConsoleFilter alloc] init];
  f->_title = [title copy];
  f->_only = [types copy];
  return f;
}

+ (instancetype)filterNamed:(NSString*)title except:(NSSet<NSString*>*)types {
  VrstConsoleFilter* f = [[VrstConsoleFilter alloc] init];
  f->_title = [title copy];
  f->_hidden = [types copy];
  return f;
}

@end

@implementation VrstDebugConsole {
  /// Live status at the top.
  UILabel* _status;
  /// One segment per filter.
  UISegmentedControl* _mode;
  /// Split / Full switch.
  UIButton* _size;
  /// Substring filter.
  UITextField* _filter;
  /// The log.
  UITextView* _text;
  /// Kept lines, oldest first.
  std::deque<Vrst_console_line> _lines;
  /// Font, color and hanging indent of the log text.
  NSDictionary<NSAttributedStringKey, id>* _logStyle;
}

/// Builds the console, hidden and closed.
/// @param frame Initial frame; VrstRootController places it.
/// @return The console.
- (instancetype)initWithFrame:(CGRect)frame {
  self = [super initWithFrame:frame];
  self.hidden = YES;
  self.clipsToBounds = YES;
  self.backgroundColor = [UIColor colorWithWhite:0.04 alpha:1];
  UIFont* mono = [UIFont monospacedSystemFontOfSize:10 weight:UIFontWeightRegular];

  UILabel* title = [[UILabel alloc] init];
  title.text = @"CONSOLE";
  title.font = [UIFont monospacedSystemFontOfSize:13 weight:UIFontWeightBold];
  title.textColor = UIColor.systemOrangeColor;
  [title setContentHuggingPriority:UILayoutPriorityDefaultLow forAxis:UILayoutConstraintAxisHorizontal];
  _size = [UIButton buttonWithType:UIButtonTypeSystem];
  [_size addTarget:self action:@selector(onSize) forControlEvents:UIControlEventTouchUpInside];
  UIButton* close = [UIButton buttonWithType:UIButtonTypeSystem];
  [close setTitle:@"Close" forState:UIControlStateNormal];
  [close addTarget:self action:@selector(close) forControlEvents:UIControlEventTouchUpInside];
  UIStackView* header = [[UIStackView alloc] initWithArrangedSubviews:@[ title, _size, close ]];
  header.spacing = 16;
  header.alignment = UIStackViewAlignmentCenter;

  _mode = [[UISegmentedControl alloc] initWithItems:@[]];
  [_mode addTarget:self action:@selector(onFilterChanged) forControlEvents:UIControlEventValueChanged];

  _status = [[UILabel alloc] init];
  _status.numberOfLines = 0;
  _status.font = mono;
  _status.textColor = UIColor.systemYellowColor;

  _filter = [[UITextField alloc] init];
  _filter.placeholder = @"filter";
  _filter.font = mono;
  _filter.textColor = UIColor.whiteColor;
  _filter.borderStyle = UITextBorderStyleRoundedRect;
  _filter.backgroundColor = [UIColor colorWithWhite:0.15 alpha:1];
  _filter.autocapitalizationType = UITextAutocapitalizationTypeNone;
  _filter.autocorrectionType = UITextAutocorrectionTypeNo;
  _filter.clearButtonMode = UITextFieldViewModeWhileEditing;
  _filter.returnKeyType = UIReturnKeyDone;
  [_filter addTarget:self action:@selector(onFilterChanged) forControlEvents:UIControlEventEditingChanged];
  [_filter addTarget:_filter action:@selector(resignFirstResponder) forControlEvents:UIControlEventEditingDidEndOnExit];

  _text = [[UITextView alloc] init];
  _text.editable = NO;
  _text.backgroundColor = UIColor.clearColor;
  _text.keyboardDismissMode = UIScrollViewKeyboardDismissModeOnDrag;
  [_text setContentCompressionResistancePriority:UILayoutPriorityDefaultLow forAxis:UILayoutConstraintAxisVertical];
  NSMutableParagraphStyle* wrap = [[NSMutableParagraphStyle alloc] init];
  wrap.headIndent = kWrapIndent;
  _logStyle = @{
    NSFontAttributeName : mono,
    NSForegroundColorAttributeName : [UIColor colorWithRed:0.55 green:1 blue:0.55 alpha:1],
    NSParagraphStyleAttributeName : wrap,
  };

  UIStackView* stack = [[UIStackView alloc] initWithArrangedSubviews:@[ header, _mode, _status, _filter, _text ]];
  stack.axis = UILayoutConstraintAxisVertical;
  stack.spacing = 6;
  stack.translatesAutoresizingMaskIntoConstraints = NO;
  [self addSubview:stack];
  UILayoutGuide* safe = self.safeAreaLayoutGuide;
  // At the console's bottom, but above the keyboard when it covers the console (Full).
  NSLayoutConstraint* bottom = [stack.bottomAnchor constraintEqualToAnchor:self.bottomAnchor constant:-6];
  bottom.priority = UILayoutPriorityDefaultHigh;
  [NSLayoutConstraint activateConstraints:@[
    [stack.topAnchor constraintEqualToAnchor:safe.topAnchor constant:6],
    bottom,
    [stack.bottomAnchor constraintLessThanOrEqualToAnchor:self.keyboardLayoutGuide.topAnchor constant:-6],
    [stack.leadingAnchor constraintEqualToAnchor:safe.leadingAnchor constant:8],
    [stack.trailingAnchor constraintEqualToAnchor:safe.trailingAnchor constant:-8],
  ]];
  return self;
}

- (void)setFilters:(NSArray<VrstConsoleFilter*>*)filters {
  _filters = [filters copy];
  [_mode removeAllSegments];
  for (VrstConsoleFilter* f in _filters) [_mode insertSegmentWithTitle:f.title atIndex:_mode.numberOfSegments animated:NO];
  _mode.selectedSegmentIndex = _filters.count > 0 ? 0 : UISegmentedControlNoSegment;
  [self redrawLog];
}

- (BOOL)shown {
  return _layout != VrstConsoleLayoutClosed;
}

- (void)show:(VrstConsoleLayout)layout {
  if (layout == _layout) return;
  const BOOL opening = _layout == VrstConsoleLayoutClosed;
  [self setLayout:layout];
  if (opening) [self redrawLog];
}

- (void)close {
  if (_layout == VrstConsoleLayoutClosed) return;
  [_filter resignFirstResponder];
  [self setLayout:VrstConsoleLayoutClosed];
  if (self.closeHandler) self.closeHandler();
}

/// Stores the layout, shows the status lines only in Full, names the Split / Full
/// button after the other layout and tells the layoutHandler.
/// @param layout New layout.
- (void)setLayout:(VrstConsoleLayout)layout {
  _layout = layout;
  // In Split the map's stats box below shows the same status: the log gets the room.
  _status.hidden = layout == VrstConsoleLayoutSplit;
  [_size setTitle:layout == VrstConsoleLayoutFull ? @"Split" : @"Full" forState:UIControlStateNormal];
  if (self.layoutHandler) self.layoutHandler(layout);
}

/// Split / Full button: switches between the two.
- (void)onSize {
  [self show:_layout == VrstConsoleLayoutFull ? VrstConsoleLayoutSplit : VrstConsoleLayoutFull];
}

- (void)setStatusLines:(NSArray<NSString*>*)lines {
  NSString* text = [lines componentsJoinedByString:@"\n"];
  if (![text isEqualToString:_status.text]) _status.text = text;
}

- (void)clearLog {
  _lines.clear();
  if (self.shown) [self redrawLog];
}

- (void)appendLogLines:(NSArray<NSString*>*)lines {
  bool visible = false;
  for (NSString* line in lines) {
    _lines.push_back({line, [VrstDebugConsole typeOf:line]});
    visible = visible || [self passes:_lines.back()];
  }
  while (_lines.size() > kKeptLines) _lines.pop_front();
  if (visible && self.shown) [self redrawLog];
}

/// Record type of a readable line: its second word.
/// @param line Readable line, `<t> <type> ...`.
/// @return Type; empty when the line has no second word.
+ (NSString*)typeOf:(NSString*)line {
  const NSRange first = [line rangeOfString:@" "];
  if (first.location == NSNotFound) return @"";
  const NSUInteger from = first.location + 1;
  const NSRange second = [line rangeOfString:@" " options:0 range:NSMakeRange(from, line.length - from)];
  const NSUInteger end = second.location == NSNotFound ? line.length : second.location;
  return [line substringWithRange:NSMakeRange(from, end - from)];
}

/// Whether a line is shown with the selected filter button and the filter text.
/// @param line Kept line.
/// @return YES when shown.
- (bool)passes:(const Vrst_console_line&)line {
  const NSInteger i = _mode.selectedSegmentIndex;
  if (i >= 0 && static_cast<NSUInteger>(i) < _filters.count) {
    VrstConsoleFilter* f = _filters[static_cast<NSUInteger>(i)];
    if (f.only && ![f.only containsObject:line.type]) return false;
    if ([f.hidden containsObject:line.type]) return false;
  }
  return _filter.text.length == 0 || [line.text localizedCaseInsensitiveContainsString:_filter.text];
}

/// Filter button or text changed: show the matching lines from the end.
- (void)onFilterChanged {
  [self redrawLog];
  [self scrollToEnd];
}

/// Sets the text to the last kShownLines matching lines. Follows the end when the
/// view was at the end; otherwise keeps the scroll position.
- (void)redrawLog {
  const CGFloat end = _text.contentSize.height - _text.bounds.size.height;
  const bool follow = _text.contentOffset.y >= end - kFollowSlack;
  const CGPoint offset = _text.contentOffset;
  NSMutableArray<NSString*>* shown = [NSMutableArray array];
  for (auto it = _lines.rbegin(); it != _lines.rend() && shown.count < kShownLines; ++it)
    if ([self passes:*it]) [shown addObject:it->text];
  _text.attributedText =
      [[NSAttributedString alloc] initWithString:[[[shown reverseObjectEnumerator] allObjects] componentsJoinedByString:@"\n"]
                                      attributes:_logStyle];
  if (follow) {
    [self scrollToEnd];
  } else {
    _text.contentOffset = offset;
  }
}

/// Scrolls the log to its last line.
- (void)scrollToEnd {
  [_text layoutIfNeeded];
  const CGFloat end = _text.contentSize.height - _text.bounds.size.height;
  _text.contentOffset = CGPointMake(0, std::max<CGFloat>(end, 0));
}

@end
