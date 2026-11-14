/// @file
/// VrstDownloadManager: makes map regions on the phone, the pipeline that used to run on
/// the Mac - download Geofabrik extracts, merge several into one, convert to nav files
/// (merge_pbf, convert_osm) - with one explicit state per region.
///
/// A job is one region the user asked for, made of one or more extracts. Jobs run in
/// order, one download and one conversion at a time. Every state change is saved
/// (Documents/regions/jobs.json), so a closed app finds its jobs again: a download
/// continues from where it stopped (URLSession resume data), a conversion starts again
/// from the downloaded files. The extracts and their outlines are kept in
/// Documents/regions/<id>.download/ until the region is made.

#import <Foundation/Foundation.h>

NS_ASSUME_NONNULL_BEGIN

/// Posted (main thread) on every change of a job: state, or progress at most 4 times a second.
extern NSNotificationName const VrstDownloadsChangedNotification;

/// State of a job, in the order they normally come.
typedef NS_ENUM(NSInteger, VrstJobState) {
  VrstJobQueued,       ///< Waiting for its turn.
  VrstJobDownloading,  ///< An extract is being downloaded.
  VrstJobPaused,       ///< Download stopped; the bytes so far are kept.
  VrstJobMerging,      ///< Several extracts merged into one file.
  VrstJobConverting,   ///< Converted to nav files.
  VrstJobReady,        ///< The region is on the phone.
  VrstJobCancelled,    ///< Cancelled by the user; files removed.
  VrstJobFailed,       ///< Stopped by an error; see reason.
};

/// One extract of a job: a Geofabrik region file and its outline.
@interface VrstExtract : NSObject
@property(nonatomic, copy) NSString* extractId;  ///< Geofabrik id, e.g. "bourgogne".
@property(nonatomic, copy) NSString* name;       ///< Display name.
@property(nonatomic, copy) NSString* pbfUrl;     ///< .osm.pbf URL.
@property(nonatomic, copy) NSString* polyUrl;    ///< .poly outline URL.
@property(nonatomic) long long received;         ///< Bytes downloaded so far.
@property(nonatomic) long long total;            ///< Size of the file; 0 = not known yet.
@property(nonatomic) BOOL done;                  ///< The file and its outline are on the phone.
@end

/// One region being made (or made, cancelled, failed).
@interface VrstDownloadJob : NSObject
@property(nonatomic, copy) NSString* regionId;  ///< Folder name of the region.
@property(nonatomic, copy) NSString* name;      ///< Region name.
@property(nonatomic, copy) NSArray<VrstExtract*>* extracts;  ///< Its extracts, in order.
@property(nonatomic) VrstJobState state;        ///< Where it is.
@property(nonatomic, copy, nullable) NSString* reason;  ///< Why it is paused or failed.
@property(nonatomic) double bytesPerSecond;     ///< Download speed (smoothed).
@property(nonatomic, copy) NSString* phase;     ///< Merging / converting: phase name ("areas").
@property(nonatomic) int phaseIndex;            ///< Merging / converting: phase number, 1-based.
@property(nonatomic) int phaseCount;            ///< Merging / converting: number of phases.
@property(nonatomic) double fraction;           ///< Merging / converting: how far the phase is, 0..1.
@property(nonatomic) BOOL resumeInForeground;   ///< Paused or stopped because the app went to the background.

/// One line for the Maps screen, e.g. "Downloading 45% - 92 of 205 MB - 3.1 MB/s - 1 min left".
/// @return Text.
- (NSString*)stateText;
/// Bytes downloaded and expected over all extracts (expected 0 = not known).
/// @param received Receives the bytes so far.
/// @param total Receives the expected bytes.
- (void)received:(long long*)received total:(long long*)total;
@end

/// The download manager; see the file comment. Main thread only.
@interface VrstDownloadManager : NSObject

/// Starts the manager over the regions folder and picks up saved jobs.
/// @param root Documents/regions.
/// @return The manager.
- (instancetype)initWithRoot:(NSString*)root;

/// Jobs that are not Ready, oldest first (Ready ones are dropped from the list).
@property(nonatomic, readonly) NSArray<VrstDownloadJob*>* jobs;

/// Called when a region was made (on the main thread).
@property(nonatomic, copy, nullable) void (^readyHandler)(NSString* regionId);
/// Called for every state change and pipeline phase, for the session log: a record type
/// and its fields.
@property(nonatomic, copy, nullable) void (^logHandler)(NSString* type, NSDictionary<NSString*, id>* fields);

/// Queues a new region.
/// @param name Region name.
/// @param extracts Its extracts (received/total/done are reset).
/// @return The job; nil when a job or region of that id exists.
- (nullable VrstDownloadJob*)addRegion:(NSString*)name extracts:(NSArray<VrstExtract*>*)extracts;

/// Pauses a downloading job (bytes kept).
/// @param job Job.
- (void)pause:(VrstDownloadJob*)job;
/// Resumes a paused job.
/// @param job Job.
- (void)resume:(VrstDownloadJob*)job;
/// Cancels a job: stops it and removes its files; it stays listed as Cancelled.
/// @param job Job.
- (void)cancel:(VrstDownloadJob*)job;
/// Queues a cancelled or failed job again (downloaded files are reused).
/// @param job Job.
- (void)retry:(VrstDownloadJob*)job;
/// Removes a cancelled or failed job from the list.
/// @param job Job.
- (void)remove:(VrstDownloadJob*)job;

/// The app goes to the background: pauses the download (bytes kept) and stops a
/// conversion; both continue when the app is back (foreground).
- (void)background;
/// The app is back: continues what background() stopped.
- (void)foreground;

/// Overall progress for the Maps button: the running job's fraction, or -1 when idle.
/// @return 0..1 or -1.
- (double)activeFraction;

@end

NS_ASSUME_NONNULL_END
