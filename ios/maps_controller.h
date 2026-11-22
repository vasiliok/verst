/// @file
/// VrstMapsController: the Maps screen, refreshed live:
/// - In progress: every region being made, with its state (waiting, downloading, paused,
///   merging, converting, cancelled, failed); a tap offers Pause / Resume / Cancel /
///   Retry / Remove as they fit;
/// - On this phone: the regions (the shown one with a checkmark); a tap switches the map
///   to it; Edit (top left) or a swipe deletes one, the shown one too, after a
///   confirmation;
/// - Download more: the Geofabrik catalog (VrstCatalogController).

#import <UIKit/UIKit.h>

#import "download_manager.h"
#import "region_store.h"

NS_ASSUME_NONNULL_BEGIN

/// The Maps screen; see the file comment. Shown inside a navigation controller.
@interface VrstMapsController : UITableViewController

/// Builds the screen.
/// @param store Regions on the phone.
/// @param manager Regions being downloaded and made.
/// @return The controller.
- (instancetype)initWithStore:(VrstRegionStore*)store manager:(VrstDownloadManager*)manager;

/// Called when the user picks a region to show; the screen closes itself first.
@property(nonatomic, copy, nullable) void (^chooseHandler)(NSString* regionId);

/// Called before a region is deleted, so the map can close it when it is the one shown
/// (its files are open).
@property(nonatomic, copy, nullable) void (^releaseHandler)(NSString* regionId);

/// Called after a region was deleted.
@property(nonatomic, copy, nullable) void (^deleteHandler)(NSString* regionId);

/// Re-reads the store and redraws the list.
- (void)reload;

@end

NS_ASSUME_NONNULL_END
