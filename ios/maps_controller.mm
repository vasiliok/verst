/// @file
/// VrstMapsController: regions being made, regions on the phone, the catalog.
/// Declarations and documentation are in maps_controller.h.

#import "maps_controller.h"

#import "catalog_controller.h"

/// Table cell reuse id.
static NSString* const kCellId = @"region";

/// Sections of the screen.
enum Vrst_maps_section : NSInteger { kSectionJobs, kSectionRegions, kSectionMore, kSectionCount };

@implementation VrstMapsController {
  /// Regions on the phone.
  VrstRegionStore* _store;
  /// Regions being made.
  VrstDownloadManager* _manager;
  /// Regions as listed, by name.
  NSArray<VrstRegion*>* _regions;
  /// Jobs as listed.
  NSArray<VrstDownloadJob*>* _jobs;
  /// Catalog list and selection, kept while the screen is open.
  VrstCatalog* _catalog;
}

- (instancetype)initWithStore:(VrstRegionStore*)store manager:(VrstDownloadManager*)manager {
  self = [super initWithStyle:UITableViewStyleInsetGrouped];
  _store = store;
  _manager = manager;
  _catalog = [[VrstCatalog alloc] init];
  self.title = @"Maps";
  return self;
}

/// Close button, the first list, live updates from the download manager.
- (void)viewDidLoad {
  [super viewDidLoad];
  self.navigationItem.rightBarButtonItem =
      [[UIBarButtonItem alloc] initWithBarButtonSystemItem:UIBarButtonSystemItemDone target:self action:@selector(onDone)];
  self.navigationItem.leftBarButtonItem = self.editButtonItem;  // Edit: a delete control on every region
  [NSNotificationCenter.defaultCenter addObserver:self selector:@selector(reload)
                                             name:VrstDownloadsChangedNotification object:nil];
  [self reload];
}

/// Stops the live updates.
- (void)dealloc {
  [NSNotificationCenter.defaultCenter removeObserver:self];
}

/// Fresh lists whenever the screen shows again (back from the catalog).
/// @param animated Animated.
- (void)viewWillAppear:(BOOL)animated {
  [super viewWillAppear:animated];
  [self reload];
}

- (void)reload {
  _regions = [_store regions];
  _jobs = _manager.jobs;
  [self.tableView reloadData];
}

/// Done button: closes the screen.
- (void)onDone {
  [self dismissViewControllerAnimated:YES completion:nil];
}

/// Three sections.
/// @param tableView Table.
/// @return kSectionCount.
- (NSInteger)numberOfSectionsInTableView:(UITableView*)tableView {
  (void)tableView;
  return kSectionCount;
}

/// Section titles.
/// @param tableView Table.
/// @param section Section.
/// @return Title.
- (NSString*)tableView:(UITableView*)tableView titleForHeaderInSection:(NSInteger)section {
  (void)tableView;
  switch (section) {
    case kSectionJobs: return _jobs.count ? @"In progress" : nil;
    case kSectionRegions: return @"On this phone";
    default: return @"Download more";
  }
}

/// Hints.
/// @param tableView Table.
/// @param section Section.
/// @return Text.
- (NSString*)tableView:(UITableView*)tableView titleForFooterInSection:(NSInteger)section {
  (void)tableView;
  switch (section) {
    case kSectionJobs: return _jobs.count ? @"Tap a row to pause, resume or cancel it." : nil;
    case kSectionRegions:
      return _regions.count == 0 ? @"No map yet."
                                 : @"Tap a region to show it. Edit (top left) or swipe left to delete one.";
    default: return @"Downloaded, merged and converted on this phone.";
  }
}

/// Rows per section.
/// @param tableView Table.
/// @param section Section.
/// @return Count.
- (NSInteger)tableView:(UITableView*)tableView numberOfRowsInSection:(NSInteger)section {
  (void)tableView;
  switch (section) {
    case kSectionJobs: return static_cast<NSInteger>(_jobs.count);
    case kSectionRegions: return static_cast<NSInteger>(_regions.count);
    default: return 1;
  }
}

/// A job (name and state), a region (name, size, date; checkmark when shown) or the
/// catalog row.
/// @param tableView Table.
/// @param indexPath Row.
/// @return Cell.
- (UITableViewCell*)tableView:(UITableView*)tableView cellForRowAtIndexPath:(NSIndexPath*)indexPath {
  UITableViewCell* cell = [tableView dequeueReusableCellWithIdentifier:kCellId];
  if (!cell) cell = [[UITableViewCell alloc] initWithStyle:UITableViewCellStyleSubtitle reuseIdentifier:kCellId];
  cell.accessoryType = UITableViewCellAccessoryNone;
  cell.detailTextLabel.numberOfLines = 0;
  const auto row = static_cast<NSUInteger>(indexPath.row);
  if (indexPath.section == kSectionJobs) {
    VrstDownloadJob* j = _jobs[row];
    cell.textLabel.text = j.name;
    cell.detailTextLabel.text = j.stateText;
    cell.detailTextLabel.textColor = j.state == VrstJobFailed      ? UIColor.systemRedColor
                                     : j.state == VrstJobPaused    ? UIColor.systemOrangeColor
                                     : j.state == VrstJobCancelled ? UIColor.secondaryLabelColor
                                                                   : UIColor.systemBlueColor;
  } else if (indexPath.section == kSectionRegions) {
    VrstRegion* r = _regions[row];
    cell.textLabel.text = r.name;
    NSString* date = [NSDateFormatter localizedStringFromDate:r.made dateStyle:NSDateFormatterMediumStyle
                                                    timeStyle:NSDateFormatterNoStyle];
    NSString* outline = r.polyFiles.count > 0 ? @"" : @" - no outline";
    cell.detailTextLabel.text = [NSString stringWithFormat:@"%.0f MB - %@%@", r.bytes / 1e6, date, outline];
    cell.detailTextLabel.textColor = UIColor.secondaryLabelColor;
    cell.accessoryType = [r.regionId isEqualToString:_store.activeId ?: @""] ? UITableViewCellAccessoryCheckmark
                                                                            : UITableViewCellAccessoryNone;
  } else {
    cell.textLabel.text = @"Geofabrik extracts";
    cell.detailTextLabel.text = @"Pick one region, or several to join into one";
    cell.detailTextLabel.textColor = UIColor.secondaryLabelColor;
    cell.accessoryType = UITableViewCellAccessoryDisclosureIndicator;
  }
  return cell;
}

/// Tap: job actions, switch region, or open the catalog.
/// @param tableView Table.
/// @param indexPath Row.
- (void)tableView:(UITableView*)tableView didSelectRowAtIndexPath:(NSIndexPath*)indexPath {
  [tableView deselectRowAtIndexPath:indexPath animated:YES];
  const auto row = static_cast<NSUInteger>(indexPath.row);
  if (indexPath.section == kSectionJobs) {
    [self actionsFor:_jobs[row] from:[tableView cellForRowAtIndexPath:indexPath]];
  } else if (indexPath.section == kSectionRegions) {
    NSString* rid = _regions[row].regionId;
    void (^choose)(NSString*) = self.chooseHandler;
    [self dismissViewControllerAnimated:YES completion:^{
      if (choose) choose(rid);
    }];
  } else {
    VrstCatalogController* c = [[VrstCatalogController alloc] initWithCatalog:_catalog parent:@""
                                                                        title:@"Geofabrik" manager:_manager];
    [self.navigationController pushViewController:c animated:YES];
  }
}

/// The actions that fit a job's state.
/// @param job Job.
/// @param source Row, for the popover on an iPad.
- (void)actionsFor:(VrstDownloadJob*)job from:(UIView*)source {
  UIAlertController* menu = [UIAlertController alertControllerWithTitle:job.name message:job.stateText
                                                         preferredStyle:UIAlertControllerStyleActionSheet];
  VrstDownloadManager* m = _manager;
  const auto add = [&](NSString* title, UIAlertActionStyle style, void (^action)(void)) {
    [menu addAction:[UIAlertAction actionWithTitle:title style:style handler:^(UIAlertAction*) { action(); }]];
  };
  switch (job.state) {
    case VrstJobQueued:
    case VrstJobDownloading: {
      add(@"Pause", UIAlertActionStyleDefault, ^{ [m pause:job]; });
      add(@"Cancel download", UIAlertActionStyleDestructive, ^{ [m cancel:job]; });
      break;
    }
    case VrstJobPaused: {
      add(@"Resume", UIAlertActionStyleDefault, ^{ [m resume:job]; });
      add(@"Cancel download", UIAlertActionStyleDestructive, ^{ [m cancel:job]; });
      break;
    }
    case VrstJobMerging:
    case VrstJobConverting: {
      add(@"Cancel", UIAlertActionStyleDestructive, ^{ [m cancel:job]; });
      break;
    }
    case VrstJobCancelled:
    case VrstJobFailed: {
      add(@"Retry", UIAlertActionStyleDefault, ^{ [m retry:job]; });
      add(@"Remove from list", UIAlertActionStyleDestructive, ^{ [m remove:job]; });
      break;
    }
    case VrstJobReady:
      break;
  }
  [menu addAction:[UIAlertAction actionWithTitle:@"Close" style:UIAlertActionStyleCancel handler:nil]];
  menu.popoverPresentationController.sourceView = source;
  [self presentViewController:menu animated:YES completion:nil];
}

/// Only regions can be deleted (Edit mode and swipe).
/// @param tableView Table.
/// @param indexPath Row.
/// @return YES for a region row.
- (BOOL)tableView:(UITableView*)tableView canEditRowAtIndexPath:(NSIndexPath*)indexPath {
  (void)tableView;
  return indexPath.section == kSectionRegions;
}

/// Swipe to delete a region, after a confirmation.
/// @param tableView Table.
/// @param indexPath Row.
/// @return Swipe actions, or nil.
- (UISwipeActionsConfiguration*)tableView:(UITableView*)tableView
    trailingSwipeActionsConfigurationForRowAtIndexPath:(NSIndexPath*)indexPath {
  if (indexPath.section != kSectionRegions) return nil;
  VrstRegion* r = _regions[static_cast<NSUInteger>(indexPath.row)];
  __weak VrstMapsController* weak = self;
  UIView* source = [tableView cellForRowAtIndexPath:indexPath];
  UIContextualAction* del = [UIContextualAction
      contextualActionWithStyle:UIContextualActionStyleDestructive
                          title:@"Delete"
                        handler:^(UIContextualAction*, UIView*, void (^done)(BOOL)) {
                          [weak confirmDelete:r from:source done:done];
                        }];
  return [UISwipeActionsConfiguration configurationWithActions:@[ del ]];
}

/// The red delete control of Edit mode: delete after a confirmation.
/// @param tableView Table.
/// @param style Always delete here.
/// @param indexPath Row.
- (void)tableView:(UITableView*)tableView
    commitEditingStyle:(UITableViewCellEditingStyle)style
     forRowAtIndexPath:(NSIndexPath*)indexPath {
  if (style != UITableViewCellEditingStyleDelete || indexPath.section != kSectionRegions) return;
  [self confirmDelete:_regions[static_cast<NSUInteger>(indexPath.row)]
                 from:[tableView cellForRowAtIndexPath:indexPath]
                 done:nil];
}

/// Asks before deleting: the region is gone until it is downloaded again.
/// @param r Region.
/// @param source Row, for the popover on an iPad.
/// @param done Called with YES when it was deleted, NO when not; may be nil.
- (void)confirmDelete:(VrstRegion*)r from:(UIView*)source done:(nullable void (^)(BOOL))done {
  const BOOL shown = [r.regionId isEqualToString:_store.activeId ?: @""];
  NSString* message = [NSString stringWithFormat:@"Frees %.0f MB. %@To use it again, download it again.", r.bytes / 1e6,
                                                 shown ? @"It is the map on screen: another one opens. " : @""];
  UIAlertController* ask = [UIAlertController alertControllerWithTitle:[NSString stringWithFormat:@"Delete %@?", r.name]
                                                               message:message
                                                        preferredStyle:UIAlertControllerStyleActionSheet];
  __weak VrstMapsController* weak = self;
  [ask addAction:[UIAlertAction actionWithTitle:@"Delete"
                                          style:UIAlertActionStyleDestructive
                                        handler:^(UIAlertAction*) {
                                          const BOOL ok = [weak deleteRegion:r];
                                          if (done) done(ok);
                                        }]];
  [ask addAction:[UIAlertAction actionWithTitle:@"Cancel"
                                          style:UIAlertActionStyleCancel
                                        handler:^(UIAlertAction*) {
                                          if (done) done(NO);
                                        }]];
  ask.popoverPresentationController.sourceView = source;
  [self presentViewController:ask animated:YES completion:nil];
}

/// Deletes a region (closing it first when it is shown) and refreshes the list.
/// @param r Region.
/// @return YES when it was deleted.
- (BOOL)deleteRegion:(VrstRegion*)r {
  if (self.releaseHandler) self.releaseHandler(r.regionId);
  NSError* error = nil;
  const BOOL ok = [_store deleteRegion:r.regionId error:&error];
  if (self.deleteHandler) self.deleteHandler(r.regionId);
  [self reload];
  if (!ok) {
    UIAlertController* fail = [UIAlertController alertControllerWithTitle:@"Not deleted"
                                                                  message:error.localizedDescription
                                                           preferredStyle:UIAlertControllerStyleAlert];
    [fail addAction:[UIAlertAction actionWithTitle:@"OK" style:UIAlertActionStyleCancel handler:nil]];
    [self presentViewController:fail animated:YES completion:nil];
  }
  return ok;
}

@end
