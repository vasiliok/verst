/// @file
/// VrstDownloadManager: download, merge and convert regions on the phone. Declarations
/// and documentation are in download_manager.h.

#import "download_manager.h"

#import <QuartzCore/QuartzCore.h>

#include <mach/mach.h>
#include <os/proc.h>

#include <algorithm>
#include <cmath>
#include <exception>
#include <memory>
#include <string>
#include <vector>

#include "osm_convert.h"
#include "pbf_merge.h"

using namespace nav;

/// RAM a conversion needs per byte of .osm.pbf (measured on the Mac: 579 MB peak for a
/// 326 MB extract), plus a fixed part for the app itself.
static constexpr double kRamPerPbfByte = 1.8;
/// Fixed RAM part of the estimate, bytes.
static constexpr double kRamBase = 150e6;
/// Seconds between two progress notifications.
static constexpr double kNotifySeconds = 0.25;

NSNotificationName const VrstDownloadsChangedNotification = @"VrstDownloadsChanged";

@implementation VrstExtract
@end

@implementation VrstDownloadJob {
 @public
  /// Progress of the running merge or conversion, shared with the worker thread.
  std::shared_ptr<Vrst_convert_progress> _progress;
}

- (void)received:(long long*)received total:(long long*)total {
  long long r = 0, t = 0;
  for (VrstExtract* e in _extracts) {
    r += e.done ? std::max(e.received, e.total) : e.received;
    t += e.total;
  }
  *received = r;
  *total = t;
}

- (NSString*)stateText {
  long long r = 0, t = 0;
  [self received:&r total:&t];
  const double mb = 1e6;
  switch (_state) {
    case VrstJobQueued:
      return _reason ? [NSString stringWithFormat:@"Waiting (%@)", _reason] : @"Waiting";
    case VrstJobDownloading: {
      NSUInteger part = 1;
      for (VrstExtract* e in _extracts)
        if (e.done) ++part;
      NSString* parts = _extracts.count > 1 ? [NSString stringWithFormat:@" part %lu of %lu", (unsigned long)std::min<NSUInteger>(part, _extracts.count), (unsigned long)_extracts.count] : @"";
      if (t <= 0) return [NSString stringWithFormat:@"Downloading%@ - %.0f MB", parts, r / mb];
      const double left = _bytesPerSecond > 0 ? (t - r) / _bytesPerSecond : 0;
      NSString* eta = left > 0 ? (left < 60 ? [NSString stringWithFormat:@" - %.0f s left", left]
                                            : [NSString stringWithFormat:@" - %.0f min left", std::ceil(left / 60)])
                               : @"";
      return [NSString stringWithFormat:@"Downloading%@ %.0f%% - %.0f of %.0f MB - %.1f MB/s%@", parts, 100.0 * r / t,
                                        r / mb, t / mb, _bytesPerSecond / mb, eta];
    }
    case VrstJobPaused:
      return [NSString stringWithFormat:@"Paused at %.0f%% (%.0f MB kept)%@", t > 0 ? 100.0 * r / t : 0.0, r / mb,
                                        _reason ? [NSString stringWithFormat:@" - %@", _reason] : @""];
    case VrstJobMerging:
      return [NSString stringWithFormat:@"Merging %lu extracts %.0f%%", (unsigned long)_extracts.count, 100 * _fraction];
    case VrstJobConverting:
      return [NSString stringWithFormat:@"Converting: %@ %.0f%% (step %d of %d)", _phase, 100 * _fraction, _phaseIndex,
                                        _phaseCount];
    case VrstJobReady:
      return @"Ready";
    case VrstJobCancelled:
      return @"Cancelled by you";
    case VrstJobFailed:
      return [NSString stringWithFormat:@"Failed: %@", _reason ?: @"unknown error"];
  }
  return @"";
}

@end

@interface VrstDownloadManager () <NSURLSessionDownloadDelegate>
@end

@implementation VrstDownloadManager {
  /// Documents/regions.
  NSString* _root;
  /// Jobs, oldest first.
  NSMutableArray<VrstDownloadJob*>* _jobs;
  /// Session of the downloads (delegate on the main queue).
  NSURLSession* _session;
  /// The running download, nil when none.
  NSURLSessionDownloadTask* _task;
  /// Its job.
  VrstDownloadJob* _taskJob;
  /// Its extract.
  VrstExtract* _taskExtract;
  /// The job being merged or converted, nil when none.
  VrstDownloadJob* _workJob;
  /// Serial queue of merges and conversions.
  dispatch_queue_t _workQueue;
  /// Progress poll while a conversion runs.
  NSTimer* _poll;
  /// When the last progress notification was sent.
  CFTimeInterval _notifiedAt;
  /// Bytes and time of the last speed sample.
  long long _speedBytes;
  CFTimeInterval _speedAt;  ///< Time of the last speed sample.
  /// The app is in the background: start nothing.
  BOOL _inBackground;
}

- (instancetype)initWithRoot:(NSString*)root {
  self = [super init];
  _root = [root copy];
  _jobs = [NSMutableArray array];
  _workQueue = dispatch_queue_create("verst.convert", DISPATCH_QUEUE_SERIAL);
  NSURLSessionConfiguration* cfg = NSURLSessionConfiguration.defaultSessionConfiguration;
  cfg.timeoutIntervalForRequest = 60;
  _session = [NSURLSession sessionWithConfiguration:cfg delegate:self delegateQueue:NSOperationQueue.mainQueue];
  [self loadJobs];
  dispatch_async(dispatch_get_main_queue(), ^{ [self next]; });
  return self;
}

- (NSArray<VrstDownloadJob*>*)jobs {
  return [_jobs copy];
}

// --- files ----------------------------------------------------------------------

/// Folder of a job's downloads.
/// @param job Job.
/// @return Documents/regions/<id>.download.
- (NSString*)workDir:(VrstDownloadJob*)job {
  return [_root stringByAppendingPathComponent:[job.regionId stringByAppendingString:@".download"]];
}
/// Downloaded extract.
/// @param e Extract.
/// @param job Its job.
/// @return Path.
- (NSString*)pbfPath:(VrstExtract*)e job:(VrstDownloadJob*)job {
  return [[self workDir:job] stringByAppendingPathComponent:[e.extractId stringByAppendingString:@".osm.pbf"]];
}
/// Saved resume data of an interrupted download.
/// @param e Extract.
/// @param job Its job.
/// @return Path.
- (NSString*)resumePath:(VrstExtract*)e job:(VrstDownloadJob*)job {
  return [[self workDir:job] stringByAppendingPathComponent:[e.extractId stringByAppendingString:@".resume"]];
}

/// Saves all jobs to jobs.json.
- (void)saveJobs {
  NSMutableArray* list = [NSMutableArray array];
  for (VrstDownloadJob* j in _jobs) {
    NSMutableArray* ex = [NSMutableArray array];
    for (VrstExtract* e in j.extracts)
      [ex addObject:@{@"id" : e.extractId, @"name" : e.name, @"pbf" : e.pbfUrl, @"poly" : e.polyUrl,
                      @"received" : @(e.received), @"total" : @(e.total), @"done" : @(e.done)}];
    [list addObject:@{@"id" : j.regionId, @"name" : j.name, @"state" : @(j.state), @"reason" : j.reason ?: @"",
                      @"extracts" : ex}];
  }
  NSData* data = [NSJSONSerialization dataWithJSONObject:list options:NSJSONWritingPrettyPrinted error:nil];
  [data writeToFile:[_root stringByAppendingPathComponent:@"jobs.json"] atomically:YES];
}

/// Reads jobs.json. A job that was running when the app ended continues: a download is
/// Paused (resumed by next()), a merge or conversion starts again (Queued).
- (void)loadJobs {
  NSData* data = [NSData dataWithContentsOfFile:[_root stringByAppendingPathComponent:@"jobs.json"]];
  NSArray* list = data ? [NSJSONSerialization JSONObjectWithData:data options:0 error:nil] : nil;
  if (![list isKindOfClass:NSArray.class]) return;
  for (NSDictionary* d in list) {
    VrstDownloadJob* j = [[VrstDownloadJob alloc] init];
    j.regionId = d[@"id"];
    j.name = d[@"name"];
    j.state = static_cast<VrstJobState>([d[@"state"] integerValue]);
    j.reason = [d[@"reason"] length] ? d[@"reason"] : nil;
    NSMutableArray* ex = [NSMutableArray array];
    for (NSDictionary* x in d[@"extracts"]) {
      VrstExtract* e = [[VrstExtract alloc] init];
      e.extractId = x[@"id"];
      e.name = x[@"name"];
      e.pbfUrl = x[@"pbf"];
      e.polyUrl = x[@"poly"];
      e.received = [x[@"received"] longLongValue];
      e.total = [x[@"total"] longLongValue];
      e.done = [x[@"done"] boolValue];
      [ex addObject:e];
    }
    j.extracts = ex;
    if (j.state == VrstJobDownloading) {
      j.state = VrstJobQueued;  // continues from its resume data
      j.reason = @"app was closed";
    }
    if (j.state == VrstJobMerging || j.state == VrstJobConverting) {
      j.state = VrstJobQueued;
      j.reason = @"converting again";
    }
    if (j.state != VrstJobReady) [_jobs addObject:j];
  }
}

// --- notifications and log ------------------------------------------------------

/// Tells the screen about a change; progress-only changes at most every kNotifySeconds.
/// @param force Send even if one was sent just now (state changes).
- (void)changed:(BOOL)force {
  const CFTimeInterval now = CACurrentMediaTime();
  if (!force && now - _notifiedAt < kNotifySeconds) return;
  _notifiedAt = now;
  [NSNotificationCenter.defaultCenter postNotificationName:VrstDownloadsChangedNotification object:self];
}

/// Moves a job to a state: saved, logged and shown.
/// @param job Job.
/// @param state New state.
/// @param reason Why (paused, failed), or nil.
- (void)set:(VrstDownloadJob*)job state:(VrstJobState)state reason:(nullable NSString*)reason {
  static NSString* const kNames[] = {@"queued",     @"downloading", @"paused",    @"merging",
                                     @"converting", @"ready",       @"cancelled", @"failed"};
  const VrstJobState old = job.state;
  job.state = state;
  job.reason = reason;
  long long r = 0, t = 0;
  [job received:&r total:&t];
  [self log:@"region" fields:@{@"event" : @"state", @"id" : job.regionId, @"from" : kNames[old], @"to" : kNames[state],
                               @"bytes" : @(r), @"total" : @(t), @"reason" : reason ?: @""}];
  [self saveJobs];
  [self changed:YES];
}

/// Sends one log record.
/// @param type Record type.
/// @param fields Fields.
- (void)log:(NSString*)type fields:(NSDictionary<NSString*, id>*)fields {
  if (_logHandler) _logHandler(type, fields);
}

// --- queue ----------------------------------------------------------------------

- (nullable VrstDownloadJob*)addRegion:(NSString*)name extracts:(NSArray<VrstExtract*>*)extracts {
  NSMutableString* rid = [NSMutableString string];
  for (VrstExtract* e in extracts) [rid appendString:rid.length ? [@"+" stringByAppendingString:e.extractId] : e.extractId];
  for (VrstDownloadJob* j in _jobs)
    if ([j.regionId isEqualToString:rid]) return nil;
  VrstDownloadJob* job = [[VrstDownloadJob alloc] init];
  job.regionId = rid;
  job.name = name;
  for (VrstExtract* e in extracts) {
    e.received = 0;
    e.total = 0;
    e.done = NO;
  }
  job.extracts = extracts;
  job.state = VrstJobQueued;
  [_jobs addObject:job];
  [self log:@"region" fields:@{@"event" : @"added", @"id" : rid, @"name" : name, @"extracts" : @(extracts.count)}];
  [self saveJobs];
  [self changed:YES];
  [self next];
  return job;
}

/// Starts whatever can run now: the first queued job's next download, or its merge and
/// conversion when all its extracts are on the phone.
- (void)next {
  if (_inBackground) return;
  for (VrstDownloadJob* j in _jobs) {
    if (j.state != VrstJobQueued) continue;
    VrstExtract* missing = nil;
    for (VrstExtract* e in j.extracts)
      if (!e.done) {
        missing = e;
        break;
      }
    if (missing) {
      if (!_task) [self download:missing job:j];
      if (_task) return;  // one download at a time
      continue;
    }
    if (!_workJob) [self make:j];
    return;
  }
}

// --- downloads ------------------------------------------------------------------

/// Starts (or continues from resume data) one extract's download.
/// @param e Extract.
/// @param job Its job.
- (void)download:(VrstExtract*)e job:(VrstDownloadJob*)job {
  [NSFileManager.defaultManager createDirectoryAtPath:[self workDir:job] withIntermediateDirectories:YES attributes:nil
                                                error:nil];
  NSData* resume = [NSData dataWithContentsOfFile:[self resumePath:e job:job]];
  _task = resume ? [_session downloadTaskWithResumeData:resume]
                 : [_session downloadTaskWithURL:[NSURL URLWithString:e.pbfUrl]];
  if (!resume) e.received = 0;
  [NSFileManager.defaultManager removeItemAtPath:[self resumePath:e job:job] error:nil];
  _taskJob = job;
  _taskExtract = e;
  _speedBytes = e.received;
  _speedAt = CACurrentMediaTime();
  job.bytesPerSecond = 0;
  [_task resume];
  [self log:@"region" fields:@{@"event" : @"download", @"id" : job.regionId, @"extract" : e.extractId,
                               @"resume" : @(resume != nil), @"url" : e.pbfUrl}];
  [self set:job state:VrstJobDownloading reason:nil];
}

/// Stops the running download keeping its bytes (resume data on disk).
/// @param state State for its job (Paused, or Queued to continue in the foreground).
/// @param reason Why.
- (void)stopDownloadAs:(VrstJobState)state reason:(nullable NSString*)reason {
  if (!_task) return;
  NSURLSessionDownloadTask* task = _task;
  VrstDownloadJob* job = _taskJob;
  VrstExtract* e = _taskExtract;
  _task = nil;
  _taskJob = nil;
  _taskExtract = nil;
  [task cancelByProducingResumeData:^(NSData* data) {
    if (data) [data writeToFile:[self resumePath:e job:job] atomically:YES];
  }];
  [self set:job state:state reason:reason];
}

/// Progress of the running download.
- (void)URLSession:(NSURLSession*)session
                 downloadTask:(NSURLSessionDownloadTask*)downloadTask
                 didWriteData:(int64_t)bytesWritten
            totalBytesWritten:(int64_t)totalBytesWritten
    totalBytesExpectedToWrite:(int64_t)totalBytesExpectedToWrite {
  (void)session;
  (void)bytesWritten;
  if (downloadTask != _task) return;
  _taskExtract.received = totalBytesWritten;
  if (totalBytesExpectedToWrite > 0) _taskExtract.total = totalBytesExpectedToWrite;
  const CFTimeInterval now = CACurrentMediaTime();
  if (now - _speedAt >= 1) {
    const double speed = (totalBytesWritten - _speedBytes) / (now - _speedAt);
    _taskJob.bytesPerSecond = _taskJob.bytesPerSecond > 0 ? 0.7 * _taskJob.bytesPerSecond + 0.3 * speed : speed;
    _speedBytes = totalBytesWritten;
    _speedAt = now;
  }
  [self changed:NO];
}

/// Same progress for a resumed download.
- (void)URLSession:(NSURLSession*)session
          downloadTask:(NSURLSessionDownloadTask*)downloadTask
     didResumeAtOffset:(int64_t)fileOffset
    expectedTotalBytes:(int64_t)expectedTotalBytes {
  (void)session;
  if (downloadTask != _task) return;
  _taskExtract.received = fileOffset;
  if (expectedTotalBytes > 0) _taskExtract.total = expectedTotalBytes;
  _speedBytes = fileOffset;
}

/// A download finished: the file is moved into the job folder (it must happen here),
/// then the outline is fetched.
- (void)URLSession:(NSURLSession*)session
                 downloadTask:(NSURLSessionDownloadTask*)downloadTask
    didFinishDownloadingToURL:(NSURL*)location {
  (void)session;
  if (downloadTask != _task) return;
  const NSInteger status = [(NSHTTPURLResponse*)downloadTask.response statusCode];
  if (status != 200) return;  // reported in didCompleteWithError
  NSString* dest = [self pbfPath:_taskExtract job:_taskJob];
  [NSFileManager.defaultManager removeItemAtPath:dest error:nil];
  [NSFileManager.defaultManager moveItemAtURL:location toURL:[NSURL fileURLWithPath:dest] error:nil];
}

/// A download ended: next step, or Paused / Failed with the reason.
- (void)URLSession:(NSURLSession*)session task:(NSURLSessionTask*)task didCompleteWithError:(nullable NSError*)error {
  (void)session;
  if (task != _task) return;  // paused or cancelled: already handled
  VrstDownloadJob* job = _taskJob;
  VrstExtract* e = _taskExtract;
  _task = nil;
  _taskJob = nil;
  _taskExtract = nil;
  const NSInteger status = [(NSHTTPURLResponse*)task.response statusCode];
  if (error) {
    // A lost connection keeps its bytes and waits for Resume.
    if (NSData* data = error.userInfo[NSURLSessionDownloadTaskResumeData])
      [data writeToFile:[self resumePath:e job:job] atomically:YES];
    [self set:job state:VrstJobPaused reason:error.code == NSURLErrorNotConnectedToInternet ||
                                                     error.code == NSURLErrorNetworkConnectionLost
                                                 ? @"connection lost"
                                                 : error.localizedDescription];
    return;
  }
  if (status != 200 || ![NSFileManager.defaultManager fileExistsAtPath:[self pbfPath:e job:job]]) {
    [self set:job state:VrstJobFailed reason:[NSString stringWithFormat:@"download of %@: HTTP %ld", e.name, (long)status]];
    [self next];
    return;
  }
  [self fetchPoly:e job:job];
}

/// Downloads an extract's outline, then marks the extract done and goes on.
/// @param e Extract.
/// @param job Its job.
- (void)fetchPoly:(VrstExtract*)e job:(VrstDownloadJob*)job {
  NSString* dest = [[self workDir:job] stringByAppendingPathComponent:[e.extractId stringByAppendingString:@".poly"]];
  [[_session dataTaskWithURL:[NSURL URLWithString:e.polyUrl]
           completionHandler:^(NSData* data, NSURLResponse* response, NSError* error) {
             // Without an outline the region still works (no border, no grid): logged only.
             const BOOL ok = !error && [(NSHTTPURLResponse*)response statusCode] == 200 && data.length > 0;
             if (ok) [data writeToFile:dest atomically:YES];
             dispatch_async(dispatch_get_main_queue(), ^{
               [self log:@"region" fields:@{@"event" : @"outline", @"id" : job.regionId, @"extract" : e.extractId,
                                            @"ok" : @(ok)}];
               e.done = YES;
               if (job.state == VrstJobDownloading) [self set:job state:VrstJobQueued reason:nil];
               [self next];
             });
           }] resume];
}

// --- merge and convert ------------------------------------------------------------

/// Merges (several extracts) and converts a job on the work queue, after checking that
/// the phone has the memory and the disk for it.
/// @param job Job with every extract downloaded.
- (void)make:(VrstDownloadJob*)job {
  long long pbf = 0;
  std::vector<std::string> inputs;
  for (VrstExtract* e in job.extracts) {
    NSString* p = [self pbfPath:e job:job];
    pbf += [NSFileManager.defaultManager attributesOfItemAtPath:p error:nil].fileSize;
    inputs.push_back(p.UTF8String);
  }
  const double need = kRamPerPbfByte * pbf + kRamBase;
  const double avail = static_cast<double>(os_proc_available_memory());
  if (avail > 0 && avail < need) {
    [self set:job state:VrstJobFailed reason:[NSString stringWithFormat:@"not enough memory (needs ~%.1f GB, %.1f GB free)",
                                                                      need / 1e9, avail / 1e9]];
    [self next];
    return;
  }
  NSDictionary* fs = [NSFileManager.defaultManager attributesOfFileSystemForPath:_root error:nil];
  const double free_bytes = [fs[NSFileSystemFreeSize] doubleValue];
  if (free_bytes > 0 && free_bytes < 3.0 * pbf) {
    [self set:job state:VrstJobFailed reason:[NSString stringWithFormat:@"disk full (needs ~%.1f GB, %.1f GB free)",
                                                                      3.0 * pbf / 1e9, free_bytes / 1e9]];
    [self next];
    return;
  }
  _workJob = job;
  auto progress = std::make_shared<Vrst_convert_progress>();
  job->_progress = progress;
  const bool merge = inputs.size() > 1;
  job.phaseCount = static_cast<int>(Vrst_convert_phase::WriteSearch) - (merge ? 0 : 1);
  [self set:job state:merge ? VrstJobMerging : VrstJobConverting reason:nil];
  _poll = [NSTimer scheduledTimerWithTimeInterval:kNotifySeconds repeats:YES block:^(NSTimer*) { [self pollWork]; }];
  NSString* work = [self workDir:job];
  NSString* out = [_root stringByAppendingPathComponent:job.regionId];
  __weak VrstDownloadManager* weak = self;
  // Phase times go to the log, with the memory the app uses at that moment.
  progress->on_phase_done([weak, rid = job.regionId](Vrst_convert_phase p, Vrst_float64 ms) {
    NSString* name = @(phase_name(p));
    const double footprint = [VrstDownloadManager footprint];
    dispatch_async(dispatch_get_main_queue(), ^{
      [weak log:@"region" fields:@{@"event" : @"phase", @"id" : rid, @"phase" : name, @"ms" : @(ms),
                                   @"footprint_mb" : @(footprint / 1048576.0)}];
    });
  });
  dispatch_async(_workQueue, ^{
    NSString* error = nil;
    BOOL cancelled = NO;
    try {
      std::string pbf_path = inputs.front();
      if (merge) {
        pbf_path = std::string([work stringByAppendingPathComponent:@"merged.osm.pbf"].UTF8String);
        merge_pbf(inputs, pbf_path, *progress);
      }
      convert_osm(pbf_path, out.UTF8String, *progress);
    } catch (const Vrst_convert_cancelled&) {
      cancelled = YES;
    } catch (const std::exception& e) {
      error = @(e.what());
    }
    dispatch_async(dispatch_get_main_queue(), ^{ [weak workDone:job error:error cancelled:cancelled]; });
  });
}

/// Physical memory the app uses now (what iOS counts against its limit).
/// @return Bytes; 0 when unknown.
+ (double)footprint {
  task_vm_info_data_t info{};
  mach_msg_type_number_t count = TASK_VM_INFO_COUNT;
  if (task_info(mach_task_self(), TASK_VM_INFO, reinterpret_cast<task_info_t>(&info), &count) != KERN_SUCCESS) return 0;
  return static_cast<double>(info.phys_footprint);
}

/// Copies the worker's phase and fraction into the job for the screen.
- (void)pollWork {
  VrstDownloadJob* job = _workJob;
  if (!job || !job->_progress) return;
  const Vrst_convert_phase p = job->_progress->phase();
  if (p == Vrst_convert_phase::Idle) return;
  const bool merging = p == Vrst_convert_phase::Merge;
  if (!merging && job.state == VrstJobMerging) [self set:job state:VrstJobConverting reason:nil];
  job.phase = @(phase_name(p));
  job.phaseIndex = static_cast<int>(p) - (job.extracts.count > 1 ? 0 : 1);
  job.fraction = job->_progress->fraction();
  [self changed:NO];
}

/// A merge and conversion ended: the region is made (outlines and region.json added,
/// downloads removed), or the job is cancelled, failed, or waits for the foreground.
/// @param job Job.
/// @param error Error text, or nil.
/// @param cancelled Stopped by cancel().
- (void)workDone:(VrstDownloadJob*)job error:(nullable NSString*)error cancelled:(BOOL)cancelled {
  [_poll invalidate];
  _poll = nil;
  _workJob = nil;
  job->_progress.reset();
  NSFileManager* fm = NSFileManager.defaultManager;
  NSString* work = [self workDir:job];
  [fm removeItemAtPath:[work stringByAppendingPathComponent:@"merged.osm.pbf"] error:nil];
  if (cancelled) {
    if (job.state == VrstJobCancelled) {
      [fm removeItemAtPath:work error:nil];
    } else {
      [self set:job state:VrstJobQueued reason:@"converting again when the app is back"];
    }
  } else if (error) {
    [self set:job state:VrstJobFailed reason:error];
  } else {
    NSString* out = [_root stringByAppendingPathComponent:job.regionId];
    NSMutableArray* names = [NSMutableArray array];
    for (VrstExtract* e in job.extracts) {
      [names addObject:e.extractId];
      NSString* poly = [work stringByAppendingPathComponent:[e.extractId stringByAppendingString:@".poly"]];
      [fm copyItemAtPath:poly toPath:[out stringByAppendingPathComponent:poly.lastPathComponent] error:nil];
    }
    NSDictionary* info = @{@"name" : job.name, @"made" : @(NSDate.date.timeIntervalSince1970), @"extracts" : names};
    [[NSJSONSerialization dataWithJSONObject:info options:NSJSONWritingPrettyPrinted error:nil]
        writeToFile:[out stringByAppendingPathComponent:@"region.json"] atomically:YES];
    [fm removeItemAtPath:work error:nil];
    [self set:job state:VrstJobReady reason:nil];
    [_jobs removeObject:job];
    [self saveJobs];
    [self changed:YES];
    if (_readyHandler) _readyHandler(job.regionId);
  }
  [self next];
}

// --- user actions -----------------------------------------------------------------

- (void)pause:(VrstDownloadJob*)job {
  if (job == _taskJob) {
    [self stopDownloadAs:VrstJobPaused reason:nil];
  } else if (job.state == VrstJobQueued) {
    [self set:job state:VrstJobPaused reason:nil];
  }
  [self next];
}

- (void)resume:(VrstDownloadJob*)job {
  if (job.state != VrstJobPaused) return;
  [self set:job state:VrstJobQueued reason:nil];
  [self next];
}

- (void)cancel:(VrstDownloadJob*)job {
  if (job == _taskJob) {
    [_task cancel];
    _task = nil;
    _taskJob = nil;
    _taskExtract = nil;
  }
  const BOOL working = job == _workJob;
  [self set:job state:VrstJobCancelled reason:nil];
  if (working) {
    job->_progress->cancel();  // workDone removes the files
  } else {
    [NSFileManager.defaultManager removeItemAtPath:[self workDir:job] error:nil];
  }
  for (VrstExtract* e in job.extracts) {
    e.done = NO;
    e.received = 0;
  }
  [self saveJobs];
  [self next];
}

- (void)retry:(VrstDownloadJob*)job {
  if (job.state != VrstJobCancelled && job.state != VrstJobFailed) return;
  [self set:job state:VrstJobQueued reason:nil];
  [self next];
}

- (void)remove:(VrstDownloadJob*)job {
  if (job.state != VrstJobCancelled && job.state != VrstJobFailed) return;
  [NSFileManager.defaultManager removeItemAtPath:[self workDir:job] error:nil];
  [_jobs removeObject:job];
  [self saveJobs];
  [self changed:YES];
}

- (void)background {
  _inBackground = YES;
  if (_taskJob) {
    _taskJob.resumeInForeground = YES;
    [self stopDownloadAs:VrstJobPaused reason:@"app in the background"];
  }
  if (_workJob) {
    _workJob.resumeInForeground = YES;
    _workJob->_progress->cancel();  // workDone queues it again
  }
}

- (void)foreground {
  _inBackground = NO;
  for (VrstDownloadJob* j in _jobs)
    if (j.resumeInForeground) {
      j.resumeInForeground = NO;
      if (j.state == VrstJobPaused) [self set:j state:VrstJobQueued reason:nil];
    }
  [self next];
}

- (double)activeFraction {
  if (_workJob) {
    const double steps = std::max(1, _workJob.phaseCount);
    return std::clamp((_workJob.phaseIndex - 1 + _workJob.fraction) / steps, 0.0, 1.0);
  }
  if (_taskJob) {
    long long r = 0, t = 0;
    [_taskJob received:&r total:&t];
    return t > 0 ? static_cast<double>(r) / static_cast<double>(t) : 0;
  }
  return -1;
}

@end
