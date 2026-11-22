/// @file
/// VrstCatalogController: Geofabrik extract list. Declarations and documentation are in
/// catalog_controller.h.

#import "catalog_controller.h"

/// Geofabrik's list of extracts, without their outlines (small).
static NSString* const kIndexUrl = @"https://download.geofabrik.de/index-v1-nogeom.json";
/// Cache file name.
static NSString* const kCacheFile = @"geofabrik-index.json";
/// The cached list is fetched again when older than this, seconds.
static constexpr double kCacheSeconds = 7 * 24 * 3600;
/// Table cell reuse id.
static NSString* const kCellId = @"extract";

@implementation VrstCatalog {
  /// Every extract: id, parent, name, pbf, poly.
  NSArray<NSDictionary*>* _entries;
}

- (instancetype)init {
  self = [super init];
  _selection = [NSMutableArray array];
  return self;
}

- (void)loadInto:(NSString*)cacheDir completion:(void (^)(NSString* _Nullable error))completion {
  if (_entries) {
    completion(nil);
    return;
  }
  NSString* cache = [cacheDir stringByAppendingPathComponent:kCacheFile];
  NSDate* date = [NSFileManager.defaultManager attributesOfItemAtPath:cache error:nil].fileModificationDate;
  if (date && -date.timeIntervalSinceNow < kCacheSeconds && [self parse:[NSData dataWithContentsOfFile:cache]]) {
    completion(nil);
    return;
  }
  [[NSURLSession.sharedSession dataTaskWithURL:[NSURL URLWithString:kIndexUrl]
                             completionHandler:^(NSData* data, NSURLResponse*, NSError* error) {
                               dispatch_async(dispatch_get_main_queue(), ^{
                                 if (!error && [self parse:data]) {
                                   [data writeToFile:cache atomically:YES];
                                   completion(nil);
                                 } else if ([self parse:[NSData dataWithContentsOfFile:cache]]) {
                                   completion(nil);  // offline: the old list
                                 } else {
                                   completion(error.localizedDescription ?: @"cannot read the Geofabrik list");
                                 }
                               });
                             }] resume];
}

/// Reads index-v1-nogeom.json: features[].properties {id, parent, name, urls.pbf}. The
/// outline URL is the pbf URL with "-latest.osm.pbf" replaced by ".poly".
/// @param data File content.
/// @return NO when it is not a usable list.
- (BOOL)parse:(nullable NSData*)data {
  if (!data) return NO;
  NSDictionary* root = [NSJSONSerialization JSONObjectWithData:data options:0 error:nil];
  if (![root isKindOfClass:NSDictionary.class]) return NO;
  // A value only when it is a string: JSON null (NSNull, e.g. "parent": null of the
  // continents) or another type counts as missing.
  const auto text = [](id v) -> NSString* { return [v isKindOfClass:NSString.class] ? v : nil; };
  NSArray* features = [root[@"features"] isKindOfClass:NSArray.class] ? root[@"features"] : @[];
  NSMutableArray* out = [NSMutableArray array];
  for (NSDictionary* f in features) {
    NSDictionary* p = [f isKindOfClass:NSDictionary.class] ? f[@"properties"] : nil;
    if (![p isKindOfClass:NSDictionary.class]) continue;
    NSDictionary* urls = [p[@"urls"] isKindOfClass:NSDictionary.class] ? p[@"urls"] : nil;
    NSString* rid = text(p[@"id"]);
    NSString* pbf = text(urls[@"pbf"]);
    if (!rid || !pbf) continue;
    NSString* poly = [pbf stringByReplacingOccurrencesOfString:@"-latest.osm.pbf" withString:@".poly"];
    [out addObject:@{@"id" : rid, @"parent" : text(p[@"parent"]) ?: @"", @"name" : text(p[@"name"]) ?: rid,
                     @"pbf" : pbf, @"poly" : poly}];
  }
  if (out.count == 0) return NO;
  _entries = out;
  return YES;
}

- (NSArray<NSDictionary*>*)childrenOf:(NSString*)parent {
  NSMutableArray* out = [NSMutableArray array];
  for (NSDictionary* e in _entries)
    if ([e[@"parent"] isEqualToString:parent]) [out addObject:e];
  [out sortUsingComparator:^NSComparisonResult(NSDictionary* a, NSDictionary* b) {
    return [a[@"name"] localizedCaseInsensitiveCompare:b[@"name"]];
  }];
  return out;
}

@end

@implementation VrstCatalogController {
  /// Shared list and selection.
  VrstCatalog* _catalog;
  /// Parent id of this level.
  NSString* _parent;
  /// Where new regions go.
  VrstDownloadManager* _manager;
  /// Rows: the parent itself first (when it has a file), then its children.
  NSArray<NSDictionary*>* _rows;
  /// Shown while the list loads or when it failed.
  NSString* _message;
}

- (instancetype)initWithCatalog:(VrstCatalog*)catalog
                         parent:(NSString*)parent
                          title:(NSString*)title
                        manager:(VrstDownloadManager*)manager {
  self = [super initWithStyle:UITableViewStyleInsetGrouped];
  _catalog = catalog;
  _parent = [parent copy];
  _manager = manager;
  self.title = title;
  return self;
}

/// Loads the list (first level) and the Download button.
- (void)viewDidLoad {
  [super viewDidLoad];
  _message = @"Loading the Geofabrik list...";
  NSString* docs = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES).firstObject;
  __weak VrstCatalogController* weak = self;
  [_catalog loadInto:[docs stringByAppendingPathComponent:@"regions"]
          completion:^(NSString* error) { [weak listLoaded:error]; }];
}

/// Shows the selection count in the Download button whenever the level appears.
/// @param animated Animated.
- (void)viewWillAppear:(BOOL)animated {
  [super viewWillAppear:animated];
  [self updateButton];
  [self.tableView reloadData];
}

/// The list is there (or not).
/// @param error Error text, or nil.
- (void)listLoaded:(nullable NSString*)error {
  _message = error;
  NSMutableArray* rows = [NSMutableArray array];
  if (!error) {
    if (_parent.length > 0)
      for (NSDictionary* e in [_catalog childrenOf:[self grandParent]])
        if ([e[@"id"] isEqualToString:_parent]) [rows addObject:e];  // "whole <parent>" first
    [rows addObjectsFromArray:[_catalog childrenOf:_parent]];
  }
  _rows = rows;
  [self.tableView reloadData];
}

/// Parent of this level's parent, to find the parent's own entry.
/// @return Id; "" at the top.
- (NSString*)grandParent {
  for (NSDictionary* e in [_catalog childrenOf:@""])
    if ([e[@"id"] isEqualToString:_parent]) return @"";
  // Search all levels: the list is small (a few hundred entries).
  NSMutableArray<NSString*>* queue = [NSMutableArray arrayWithObject:@""];
  while (queue.count) {
    NSString* p = queue.firstObject;
    [queue removeObjectAtIndex:0];
    for (NSDictionary* e in [_catalog childrenOf:p]) {
      if ([e[@"id"] isEqualToString:_parent]) return p;
      [queue addObject:e[@"id"]];
    }
  }
  return @"";
}

/// Download button with the number of selected extracts.
- (void)updateButton {
  const NSUInteger n = _catalog.selection.count;
  self.navigationItem.rightBarButtonItem =
      n == 0 ? nil
             : [[UIBarButtonItem alloc] initWithTitle:[NSString stringWithFormat:@"Download (%lu)", (unsigned long)n]
                                                style:UIBarButtonItemStyleDone
                                               target:self
                                               action:@selector(onDownload)];
}

/// Download: asks for the region name (default: the names joined), queues it and goes
/// back to the Maps screen.
- (void)onDownload {
  NSArray<NSDictionary*>* sel = [_catalog.selection copy];
  NSMutableArray<NSString*>* names = [NSMutableArray array];
  for (NSDictionary* e in sel) [names addObject:e[@"name"]];
  UIAlertController* ask = [UIAlertController alertControllerWithTitle:@"New region"
                                                               message:@"Downloaded, merged and converted on this phone."
                                                        preferredStyle:UIAlertControllerStyleAlert];
  [ask addTextFieldWithConfigurationHandler:^(UITextField* f) { f.text = [names componentsJoinedByString:@" + "]; }];
  __weak VrstCatalogController* weak = self;
  [ask addAction:[UIAlertAction actionWithTitle:@"Cancel" style:UIAlertActionStyleCancel handler:nil]];
  [ask addAction:[UIAlertAction actionWithTitle:@"Download"
                                          style:UIAlertActionStyleDefault
                                        handler:^(UIAlertAction*) {
                                          [weak queue:sel name:ask.textFields.firstObject.text];
                                        }]];
  [self presentViewController:ask animated:YES completion:nil];
}

/// Queues the selected extracts as one region.
/// @param sel Selected entries.
/// @param name Region name.
- (void)queue:(NSArray<NSDictionary*>*)sel name:(NSString*)name {
  NSMutableArray<VrstExtract*>* extracts = [NSMutableArray array];
  for (NSDictionary* d in sel) {
    VrstExtract* e = [[VrstExtract alloc] init];
    e.extractId = d[@"id"];
    e.name = d[@"name"];
    e.pbfUrl = d[@"pbf"];
    e.polyUrl = d[@"poly"];
    [extracts addObject:e];
  }
  if (![_manager addRegion:name.length ? name : sel.firstObject[@"name"] extracts:extracts]) {
    UIAlertController* a = [UIAlertController alertControllerWithTitle:@"Already in the list" message:nil
                                                         preferredStyle:UIAlertControllerStyleAlert];
    [a addAction:[UIAlertAction actionWithTitle:@"OK" style:UIAlertActionStyleDefault handler:nil]];
    [self presentViewController:a animated:YES completion:nil];
    return;
  }
  [_catalog.selection removeAllObjects];
  [self.navigationController popToRootViewControllerAnimated:YES];
}

/// One section.
/// @param tableView Table.
/// @return 1.
- (NSInteger)numberOfSectionsInTableView:(UITableView*)tableView {
  (void)tableView;
  return 1;
}

/// Loading message or hint.
/// @param tableView Table.
/// @param section Section.
/// @return Text.
- (NSString*)tableView:(UITableView*)tableView titleForFooterInSection:(NSInteger)section {
  (void)tableView;
  (void)section;
  return _message ?: @"Tap to select (several make one region), > to open. Geofabrik extracts.";
}

/// Rows.
/// @param tableView Table.
/// @param section Section.
/// @return Count.
- (NSInteger)tableView:(UITableView*)tableView numberOfRowsInSection:(NSInteger)section {
  (void)tableView;
  (void)section;
  return static_cast<NSInteger>(_rows.count);
}

/// An extract: checkmark when selected, a detail button when it has sub-regions.
/// @param tableView Table.
/// @param indexPath Row.
/// @return Cell.
- (UITableViewCell*)tableView:(UITableView*)tableView cellForRowAtIndexPath:(NSIndexPath*)indexPath {
  UITableViewCell* cell = [tableView dequeueReusableCellWithIdentifier:kCellId];
  if (!cell) cell = [[UITableViewCell alloc] initWithStyle:UITableViewCellStyleDefault reuseIdentifier:kCellId];
  NSDictionary* e = _rows[static_cast<NSUInteger>(indexPath.row)];
  const BOOL whole = [e[@"id"] isEqualToString:_parent];
  cell.textLabel.text = whole ? [@"All of " stringByAppendingString:e[@"name"]] : e[@"name"];
  const BOOL selected = [_catalog.selection containsObject:e];
  const BOOL children = !whole && [_catalog childrenOf:e[@"id"]].count > 0;
  cell.accessoryType = children ? UITableViewCellAccessoryDetailDisclosureButton
                                : (selected ? UITableViewCellAccessoryCheckmark : UITableViewCellAccessoryNone);
  cell.textLabel.textColor = selected ? UIColor.systemBlueColor : UIColor.labelColor;
  return cell;
}

/// Tap: open a level with sub-regions, else select / unselect.
/// @param tableView Table.
/// @param indexPath Row.
- (void)tableView:(UITableView*)tableView didSelectRowAtIndexPath:(NSIndexPath*)indexPath {
  [tableView deselectRowAtIndexPath:indexPath animated:YES];
  NSDictionary* e = _rows[static_cast<NSUInteger>(indexPath.row)];
  const BOOL whole = [e[@"id"] isEqualToString:_parent];
  if (!whole && [_catalog childrenOf:e[@"id"]].count > 0) {
    [self open:e];
    return;
  }
  if ([_catalog.selection containsObject:e]) {
    [_catalog.selection removeObject:e];
  } else {
    [_catalog.selection addObject:e];
  }
  [self updateButton];
  [tableView reloadRowsAtIndexPaths:@[ indexPath ] withRowAnimation:UITableViewRowAnimationNone];
}

/// The detail button also opens the level.
/// @param tableView Table.
/// @param indexPath Row.
- (void)tableView:(UITableView*)tableView accessoryButtonTappedForRowWithIndexPath:(NSIndexPath*)indexPath {
  (void)tableView;
  [self open:_rows[static_cast<NSUInteger>(indexPath.row)]];
}

/// Opens the sub-regions of an extract.
/// @param e Extract.
- (void)open:(NSDictionary*)e {
  VrstCatalogController* c = [[VrstCatalogController alloc] initWithCatalog:_catalog parent:e[@"id"]
                                                                      title:e[@"name"] manager:_manager];
  [self.navigationController pushViewController:c animated:YES];
}

@end
