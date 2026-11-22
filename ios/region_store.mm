/// @file
/// VrstRegionStore: regions in Documents/regions. Declarations and documentation are in
/// region_store.h.

#import "region_store.h"

/// File in the store root that holds the active region id.
static NSString* const kActiveFile = @"active.txt";
/// Per-region description file.
static NSString* const kRegionJson = @"region.json";
/// User defaults key prefix of the saved cameras.
static NSString* const kCameraKey = @"verst.camera.";

@interface VrstRegion ()
/// Fills a region from its folder.
/// @param path Region folder.
/// @return The region, or nil when the folder is not a complete region.
- (nullable instancetype)initWithPath:(NSString*)path;
@end

@implementation VrstRegion

- (nullable instancetype)initWithPath:(NSString*)path {
  NSFileManager* fm = NSFileManager.defaultManager;
  if (![fm fileExistsAtPath:[path stringByAppendingPathComponent:@"graph.dat"]]) return nil;
  self = [super init];
  _regionId = [path.lastPathComponent copy];
  _path = [path copy];
  NSDictionary* info = nil;
  if (NSData* json = [NSData dataWithContentsOfFile:[path stringByAppendingPathComponent:kRegionJson]])
    info = [NSJSONSerialization JSONObjectWithData:json options:0 error:nil];
  NSString* name = [info isKindOfClass:NSDictionary.class] ? info[@"name"] : nil;
  _name = [name isKindOfClass:NSString.class] ? [name copy] : _regionId;
  NSDate* made = [fm attributesOfItemAtPath:path error:nil].fileModificationDate ?: NSDate.date;
  if (NSNumber* t = [info isKindOfClass:NSDictionary.class] ? info[@"made"] : nil; [t isKindOfClass:NSNumber.class])
    made = [NSDate dateWithTimeIntervalSince1970:t.doubleValue];
  _made = made;
  unsigned long long bytes = 0;
  NSMutableArray<NSString*>* polys = [NSMutableArray array];
  for (NSString* f in [[fm contentsOfDirectoryAtPath:path error:nil] sortedArrayUsingSelector:@selector(compare:)]) {
    NSString* full = [path stringByAppendingPathComponent:f];
    bytes += [fm attributesOfItemAtPath:full error:nil].fileSize;
    if ([f.pathExtension isEqualToString:@"poly"]) [polys addObject:full];
  }
  _bytes = bytes;
  _polyFiles = polys;
  return self;
}

@end

@implementation VrstRegionStore

- (instancetype)initWithRoot:(NSString*)root {
  self = [super init];
  _root = [root copy];
  [NSFileManager.defaultManager createDirectoryAtPath:root withIntermediateDirectories:YES attributes:nil error:nil];
  NSString* active = [NSString stringWithContentsOfFile:[root stringByAppendingPathComponent:kActiveFile]
                                               encoding:NSUTF8StringEncoding error:nil];
  _activeId = [active stringByTrimmingCharactersInSet:NSCharacterSet.whitespaceAndNewlineCharacterSet];
  if (_activeId.length == 0) _activeId = nil;
  return self;
}

- (void)setActiveId:(nullable NSString*)activeId {
  _activeId = [activeId copy];
  NSString* file = [_root stringByAppendingPathComponent:kActiveFile];
  if (activeId) {
    [activeId writeToFile:file atomically:YES encoding:NSUTF8StringEncoding error:nil];
  } else {
    [NSFileManager.defaultManager removeItemAtPath:file error:nil];
  }
}

- (NSArray<VrstRegion*>*)regions {
  NSMutableArray<VrstRegion*>* out = [NSMutableArray array];
  for (NSString* f in [NSFileManager.defaultManager contentsOfDirectoryAtPath:_root error:nil]) {
    if ([f hasSuffix:@".tmp"]) continue;  // a region still being made
    if (VrstRegion* r = [[VrstRegion alloc] initWithPath:[_root stringByAppendingPathComponent:f]]) [out addObject:r];
  }
  [out sortUsingComparator:^NSComparisonResult(VrstRegion* a, VrstRegion* b) {
    return [a.name localizedCaseInsensitiveCompare:b.name];
  }];
  return out;
}

- (nullable VrstRegion*)regionWithId:(NSString*)regionId {
  return [[VrstRegion alloc] initWithPath:[_root stringByAppendingPathComponent:regionId]];
}

- (nullable VrstRegion*)importFolder:(NSString*)folder error:(NSError**)error {
  NSFileManager* fm = NSFileManager.defaultManager;
  NSString* rid = folder.lastPathComponent;
  NSString* dest = [_root stringByAppendingPathComponent:rid];
  NSString* tmp = [dest stringByAppendingString:@".tmp"];
  [fm removeItemAtPath:tmp error:nil];
  if (![fm createDirectoryAtPath:tmp withIntermediateDirectories:YES attributes:nil error:error]) return nil;
  for (NSString* f in [fm contentsOfDirectoryAtPath:folder error:nil]) {
    NSString* ext = f.pathExtension;
    // Nav files, outlines, the address search database (search.db) and region.json.
    if (![ext isEqualToString:@"dat"] && ![ext isEqualToString:@"poly"] && ![ext isEqualToString:@"db"] &&
        ![f isEqualToString:kRegionJson])
      continue;
    if (![fm copyItemAtPath:[folder stringByAppendingPathComponent:f] toPath:[tmp stringByAppendingPathComponent:f]
                      error:error]) {
      [fm removeItemAtPath:tmp error:nil];
      return nil;
    }
  }
  // Swap in only a complete copy.
  [fm removeItemAtPath:dest error:nil];
  if (![fm moveItemAtPath:tmp toPath:dest error:error]) return nil;
  return [self regionWithId:rid];
}

- (BOOL)deleteRegion:(NSString*)regionId error:(NSError**)error {
  [NSUserDefaults.standardUserDefaults removeObjectForKey:[kCameraKey stringByAppendingString:regionId]];
  if ([regionId isEqualToString:_activeId ?: @""]) self.activeId = nil;
  return [NSFileManager.defaultManager removeItemAtPath:[_root stringByAppendingPathComponent:regionId] error:error];
}

- (void)saveCamera:(VrstRegionCamera)camera forRegion:(NSString*)regionId {
  [NSUserDefaults.standardUserDefaults setObject:@[ @(camera.lat), @(camera.lon), @(camera.mpp), @(camera.angle) ]
                                          forKey:[kCameraKey stringByAppendingString:regionId]];
}

- (BOOL)camera:(VrstRegionCamera*)camera forRegion:(NSString*)regionId {
  NSArray* a = [NSUserDefaults.standardUserDefaults arrayForKey:[kCameraKey stringByAppendingString:regionId]];
  if (a.count != 4) return NO;
  *camera = {[a[0] doubleValue], [a[1] doubleValue], [a[2] doubleValue], [a[3] doubleValue]};
  return camera->mpp > 0;
}

@end
