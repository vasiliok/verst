/// @file
/// VrstPinCard: labels, mode switch and route buttons. Declarations and documentation
/// are in pin_card.h.

#import "pin_card.h"

@implementation VrstPinCard {
  /// First line.
  UILabel* _titleLabel;
  /// Second line.
  UILabel* _detailLabel;
  /// Car / Bike / Walk.
  UISegmentedControl* _modes;
  /// Start here button.
  UIButton* _start;
}

/// A rounded dark card: title and close button on top, the detail line, then the mode
/// switch and the Start here and Route buttons.
/// @param frame Initial frame; Auto Layout places the card.
/// @return The card.
- (instancetype)initWithFrame:(CGRect)frame {
  self = [super initWithFrame:frame];
  self.backgroundColor = [UIColor colorWithWhite:0.08 alpha:0.94];
  self.layer.cornerRadius = 14;
  self.overrideUserInterfaceStyle = UIUserInterfaceStyleDark;  // light text and controls on the dark card
  _titleLabel = [[UILabel alloc] init];
  _titleLabel.font = [UIFont boldSystemFontOfSize:17];
  _titleLabel.textColor = UIColor.whiteColor;
  _detailLabel = [[UILabel alloc] init];
  _detailLabel.font = [UIFont systemFontOfSize:14];
  _detailLabel.textColor = [UIColor colorWithWhite:0.8 alpha:1];
  _detailLabel.numberOfLines = 2;
  UIButton* close = [UIButton buttonWithType:UIButtonTypeClose];
  [close addTarget:self action:@selector(onClose) forControlEvents:UIControlEventTouchUpInside];
  [close setContentHuggingPriority:UILayoutPriorityRequired forAxis:UILayoutConstraintAxisHorizontal];
  UIStackView* top = [[UIStackView alloc] initWithArrangedSubviews:@[ _titleLabel, close ]];
  top.spacing = 8;
  _modes = [[UISegmentedControl alloc] initWithItems:@[ @"Car", @"Bike", @"Walk" ]];
  _modes.selectedSegmentIndex = 0;
  [_modes addTarget:self action:@selector(onMode) forControlEvents:UIControlEventValueChanged];
  _start = [UIButton buttonWithType:UIButtonTypeSystem];
  [_start setTitle:@"Start here" forState:UIControlStateNormal];
  [_start addTarget:self action:@selector(onStart) forControlEvents:UIControlEventTouchUpInside];
  UIButton* route = [UIButton buttonWithType:UIButtonTypeSystem];
  [route setTitle:@"Route" forState:UIControlStateNormal];
  route.titleLabel.font = [UIFont boldSystemFontOfSize:17];
  route.backgroundColor = UIColor.systemBlueColor;
  [route setTitleColor:UIColor.whiteColor forState:UIControlStateNormal];
  route.layer.cornerRadius = 8;
  route.contentEdgeInsets = UIEdgeInsetsMake(6, 18, 6, 18);
  [route addTarget:self action:@selector(onRoute) forControlEvents:UIControlEventTouchUpInside];
  UIStackView* bottom = [[UIStackView alloc] initWithArrangedSubviews:@[ _modes, _start, route ]];
  bottom.spacing = 10;
  bottom.alignment = UIStackViewAlignmentCenter;
  UIStackView* stack = [[UIStackView alloc] initWithArrangedSubviews:@[ top, _detailLabel, bottom ]];
  stack.axis = UILayoutConstraintAxisVertical;
  stack.spacing = 8;
  stack.translatesAutoresizingMaskIntoConstraints = NO;
  [self addSubview:stack];
  [NSLayoutConstraint activateConstraints:@[
    [stack.topAnchor constraintEqualToAnchor:self.topAnchor constant:12],
    [stack.bottomAnchor constraintEqualToAnchor:self.bottomAnchor constant:-12],
    [stack.leadingAnchor constraintEqualToAnchor:self.leadingAnchor constant:14],
    [stack.trailingAnchor constraintEqualToAnchor:self.trailingAnchor constant:-14],
  ]];
  self.hidden = YES;
  return self;
}

- (NSString*)title {
  return _titleLabel.text ?: @"";
}

- (void)setTitle:(NSString*)title {
  _titleLabel.text = title;
}

- (NSString*)detail {
  return _detailLabel.text ?: @"";
}

- (void)setDetail:(NSString*)detail {
  _detailLabel.text = detail;
}

- (VrstCardMode)mode {
  return static_cast<VrstCardMode>(_modes.selectedSegmentIndex);
}

- (void)setMode:(VrstCardMode)mode {
  _modes.selectedSegmentIndex = mode;
}

- (BOOL)startOffered {
  return !_start.hidden;
}

- (void)setStartOffered:(BOOL)startOffered {
  _start.hidden = !startOffered;
}

/// Route button.
- (void)onRoute {
  if (self.routeHandler) self.routeHandler();
}

/// Start here button.
- (void)onStart {
  if (self.startHandler) self.startHandler();
}

/// Close button.
- (void)onClose {
  if (self.closeHandler) self.closeHandler();
}

/// The mode switch changed.
- (void)onMode {
  if (self.modeHandler) self.modeHandler(self.mode);
}

@end
