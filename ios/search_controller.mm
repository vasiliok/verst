/// @file
/// VrstSearchController: search field, results as you type, clear pin.
/// Declarations and documentation are in search_controller.h.

#import "search_controller.h"

/// Table cell reuse id.
static NSString* const kCellId = @"hit";
/// Seconds without typing before the search runs.
static const NSTimeInterval kDebounceSeconds = 0.15;

/// Sections of the screen.
enum Vrst_search_section : NSInteger { kSectionPin, kSectionHits, kSectionCount };

@implementation VrstSearchHit
@end

@implementation VrstSearchController {
  /// Search field in the navigation bar.
  UISearchBar* _bar;
  /// Results of the last search.
  NSArray<VrstSearchHit*>* _hits;
  /// The map has an address index.
  BOOL _hasIndex;
  /// A pin is shown.
  BOOL _hasPin;
  /// Text to start with.
  NSString* _startQuery;
}

- (instancetype)initWithQuery:(NSString*)query hasIndex:(BOOL)hasIndex hasPin:(BOOL)hasPin {
  self = [super initWithStyle:UITableViewStyleInsetGrouped];
  _startQuery = [query copy];
  _hasIndex = hasIndex;
  _hasPin = hasPin;
  _hits = @[];
  return self;
}

/// Search field, Done button, first results for the starting text.
- (void)viewDidLoad {
  [super viewDidLoad];
  _bar = [[UISearchBar alloc] init];
  _bar.placeholder = @"Address, street or place";
  _bar.delegate = self;
  _bar.text = _startQuery;
  _bar.autocorrectionType = UITextAutocorrectionTypeNo;
  _bar.autocapitalizationType = UITextAutocapitalizationTypeNone;
  _bar.returnKeyType = UIReturnKeySearch;
  self.navigationItem.titleView = _bar;
  self.navigationItem.rightBarButtonItem =
      [[UIBarButtonItem alloc] initWithBarButtonSystemItem:UIBarButtonSystemItemDone target:self action:@selector(onDone)];
  self.tableView.keyboardDismissMode = UIScrollViewKeyboardDismissModeOnDrag;
  [self runSearch];
}

/// Keyboard up at once.
/// @param animated Animated.
- (void)viewDidAppear:(BOOL)animated {
  [super viewDidAppear:animated];
  [_bar becomeFirstResponder];
}

- (NSString*)query {
  return _bar.text ?: @"";
}

/// Done: closes the screen.
- (void)onDone {
  [self dismissViewControllerAnimated:YES completion:nil];
}

/// Searches the text in the field and shows the results.
- (void)runSearch {
  _hits = self.searchHandler ? self.searchHandler(self.query) : @[];
  [self.tableView reloadData];
}

/// Text changed: search when the typing pauses.
/// @param searchBar The field.
/// @param searchText New text.
- (void)searchBar:(UISearchBar*)searchBar textDidChange:(NSString*)searchText {
  [NSObject cancelPreviousPerformRequestsWithTarget:self selector:@selector(runSearch) object:nil];
  [self performSelector:@selector(runSearch) withObject:nil afterDelay:kDebounceSeconds];
}

/// Search key: search now and hide the keyboard.
/// @param searchBar The field.
- (void)searchBarSearchButtonClicked:(UISearchBar*)searchBar {
  [NSObject cancelPreviousPerformRequestsWithTarget:self selector:@selector(runSearch) object:nil];
  [self runSearch];
  [searchBar resignFirstResponder];
}

/// @param tableView The table.
/// @return Number of sections.
- (NSInteger)numberOfSectionsInTableView:(UITableView*)tableView {
  return kSectionCount;
}

/// @param tableView The table.
/// @param section Section.
/// @return Rows in it.
- (NSInteger)tableView:(UITableView*)tableView numberOfRowsInSection:(NSInteger)section {
  return section == kSectionPin ? (_hasPin ? 1 : 0) : static_cast<NSInteger>(_hits.count);
}

/// Why the list is empty, under it.
/// @param tableView The table.
/// @param section Section.
/// @return Footer text, or nil.
- (NSString*)tableView:(UITableView*)tableView titleForFooterInSection:(NSInteger)section {
  if (section != kSectionHits) return nil;
  if (!_hasIndex) return @"This map has no address index - download it again.";
  if (_hits.count == 0 && self.query.length > 0) return @"Nothing found.";
  if (_hits.count == 0) return @"For example: 12 rue de la paix dijon";
  return nil;
}

/// @param tableView The table.
/// @param indexPath Row.
/// @return The cell: the clear row, or a result with its kind, title and detail.
- (UITableViewCell*)tableView:(UITableView*)tableView cellForRowAtIndexPath:(NSIndexPath*)indexPath {
  UITableViewCell* cell = [tableView dequeueReusableCellWithIdentifier:kCellId];
  if (!cell) cell = [[UITableViewCell alloc] initWithStyle:UITableViewCellStyleSubtitle reuseIdentifier:kCellId];
  if (indexPath.section == kSectionPin) {
    cell.textLabel.text = @"Clear pin";
    cell.textLabel.textColor = UIColor.systemRedColor;
    cell.detailTextLabel.text = nil;
    cell.imageView.image = [UIImage systemImageNamed:@"mappin.slash"];
    return cell;
  }
  VrstSearchHit* h = _hits[static_cast<NSUInteger>(indexPath.row)];
  cell.textLabel.text = h.title;
  cell.textLabel.textColor = UIColor.labelColor;
  cell.detailTextLabel.text = h.detail;
  NSString* symbol = [h.kind isEqualToString:@"address"] ? @"house" : [h.kind isEqualToString:@"place"] ? @"building.2" : @"road.lanes";
  cell.imageView.image = [UIImage systemImageNamed:symbol];
  return cell;
}

/// A tap: close, then show the result or clear the pin.
/// @param tableView The table.
/// @param indexPath Row.
- (void)tableView:(UITableView*)tableView didSelectRowAtIndexPath:(NSIndexPath*)indexPath {
  [tableView deselectRowAtIndexPath:indexPath animated:YES];
  if (indexPath.section == kSectionPin) {
    void (^clear)(void) = self.clearPinHandler;
    [self dismissViewControllerAnimated:YES completion:^{ if (clear) clear(); }];
    return;
  }
  VrstSearchHit* h = _hits[static_cast<NSUInteger>(indexPath.row)];
  void (^pick)(VrstSearchHit*) = self.pickHandler;
  [self dismissViewControllerAnimated:YES completion:^{ if (pick) pick(h); }];
}

@end
