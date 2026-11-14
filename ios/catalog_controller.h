/// @file
/// VrstCatalogController: browse the Geofabrik extracts (continent > country > region),
/// select one or several, and queue them as one new region. The extract list is
/// Geofabrik's index-v1-nogeom.json, cached in Documents/regions for a week.

#import <UIKit/UIKit.h>

#import "download_manager.h"

NS_ASSUME_NONNULL_BEGIN

/// Shared state of the catalog screens: the extract list and the selection.
@interface VrstCatalog : NSObject
/// Loads the cached list, or downloads it (completion on the main thread).
/// @param cacheDir Folder of the cache file.
/// @param completion Called with nil or an error text.
- (void)loadInto:(NSString*)cacheDir completion:(void (^)(NSString* _Nullable error))completion;
/// Extracts whose parent is the given id ("" = the top level), by name.
/// @param parent Parent id.
/// @return Entries: dictionaries with id, name, pbf, poly.
- (NSArray<NSDictionary*>*)childrenOf:(NSString*)parent;
/// Selected extracts, in the order they were picked.
@property(nonatomic, readonly) NSMutableArray<NSDictionary*>* selection;
@end

/// One level of the catalog; see the file comment.
@interface VrstCatalogController : UITableViewController
/// Builds one level.
/// @param catalog Shared list and selection.
/// @param parent Parent id; "" for the top level.
/// @param title Screen title.
/// @param manager Where the new region is queued.
/// @return The controller.
- (instancetype)initWithCatalog:(VrstCatalog*)catalog
                         parent:(NSString*)parent
                          title:(NSString*)title
                        manager:(VrstDownloadManager*)manager;
@end

NS_ASSUME_NONNULL_END
