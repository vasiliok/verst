/// @file
/// VrstRegionStore: the map regions stored on the phone, in Documents/regions.
///
/// A region is one folder Documents/regions/<id>/ with the nav files (graph, edges,
/// geom, names, map .dat), the .poly outline of every extract it was made from, and
/// region.json (name and how it was made). The store lists the complete regions,
/// remembers which one is shown (active.txt) and the camera of each region, imports a
/// converted folder (`--import-region` at launch, for development) and deletes regions.

#import <Foundation/Foundation.h>

NS_ASSUME_NONNULL_BEGIN

/// One region on the phone.
@interface VrstRegion : NSObject
/// Folder name; stable id.
@property(nonatomic, readonly, copy) NSString* regionId;
/// Name to show (region.json "name", else the folder name).
@property(nonatomic, readonly, copy) NSString* name;
/// The region folder.
@property(nonatomic, readonly, copy) NSString* path;
/// Bytes of all its files.
@property(nonatomic, readonly) unsigned long long bytes;
/// When it was made (region.json "made", else the folder date).
@property(nonatomic, readonly, copy) NSDate* made;
/// Its .poly outline files, sorted; empty when there are none.
@property(nonatomic, readonly, copy) NSArray<NSString*>* polyFiles;
@end

/// Camera of a region, restored when the user switches back to it.
typedef struct {
  double lat;    ///< Screen center latitude.
  double lon;    ///< Screen center longitude.
  double mpp;    ///< Ground metres per screen point.
  double angle;  ///< Map rotation, radians counter-clockwise.
} VrstRegionCamera;

/// The regions on the phone; see the file comment.
@interface VrstRegionStore : NSObject

/// Opens (and creates) the store.
/// @param root Folder of the regions (Documents/regions).
/// @return The store.
- (instancetype)initWithRoot:(NSString*)root;

/// The folder of the regions.
@property(nonatomic, readonly, copy) NSString* root;

/// Id of the region shown, saved across launches; nil = none chosen yet.
@property(nonatomic, copy, nullable) NSString* activeId;

/// Complete regions (a folder with graph.dat), by name.
/// @return Regions.
- (NSArray<VrstRegion*>*)regions;

/// One region.
/// @param regionId Folder name.
/// @return The region, or nil when there is no complete region of that id.
- (nullable VrstRegion*)regionWithId:(NSString*)regionId;

/// Copies a converted folder (the .dat files, .poly files and region.json if any) into
/// the store as region <folder name>, replacing one of the same id.
/// @param folder Folder made by osm2nav (Simulator: a path on the Mac).
/// @param error Set on failure.
/// @return The new region, or nil on failure.
- (nullable VrstRegion*)importFolder:(NSString*)folder error:(NSError**)error;

/// Deletes a region and its camera.
/// @param regionId Folder name.
/// @param error Set on failure.
/// @return YES when it is gone.
- (BOOL)deleteRegion:(NSString*)regionId error:(NSError**)error;

/// Saves the camera of a region.
/// @param camera Camera.
/// @param regionId Folder name.
- (void)saveCamera:(VrstRegionCamera)camera forRegion:(NSString*)regionId;

/// The saved camera of a region.
/// @param regionId Folder name.
/// @param camera Receives the camera.
/// @return NO when none was saved.
- (BOOL)camera:(VrstRegionCamera*)camera forRegion:(NSString*)regionId;

@end

NS_ASSUME_NONNULL_END
