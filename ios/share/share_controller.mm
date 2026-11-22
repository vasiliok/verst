/// @file
/// VrstShareController: reads what was shared, finds the point, opens the app.
/// Declarations and documentation are in share_controller.h.

#import "share_controller.h"

#import <UniformTypeIdentifiers/UniformTypeIdentifiers.h>
#import <objc/message.h>

#include <string>

#include "geo_link.h"

/// Longest "items" text passed to the app for its log.
static const NSUInteger kMaxItemsText = 600;
/// Seconds the panel stays after the app was asked to open.
static const double kCloseSeconds = 0.4;

@implementation VrstShareController {
  /// What the panel says.
  UILabel* _status;
  /// The text to show in the app; also what Copy copies.
  NSString* _result;
  /// Copy button, shown when the app could not be opened.
  UIButton* _copy;
}

/// The panel: a title, a status line, Copy (hidden) and Cancel; then reads the items.
- (void)viewDidLoad {
  [super viewDidLoad];
  self.view.backgroundColor = UIColor.systemBackgroundColor;
  UILabel* title = [[UILabel alloc] init];
  title.text = @"Verst";
  title.font = [UIFont boldSystemFontOfSize:20];
  _status = [[UILabel alloc] init];
  _status.text = @"Reading what was shared...";
  _status.numberOfLines = 0;
  _status.textAlignment = NSTextAlignmentCenter;
  _copy = [UIButton buttonWithType:UIButtonTypeSystem];
  [_copy setTitle:@"Copy, then paste in Verst search" forState:UIControlStateNormal];
  [_copy addTarget:self action:@selector(onCopy) forControlEvents:UIControlEventTouchUpInside];
  _copy.hidden = YES;
  UIButton* cancel = [UIButton buttonWithType:UIButtonTypeSystem];
  [cancel setTitle:@"Cancel" forState:UIControlStateNormal];
  [cancel addTarget:self action:@selector(onCancel) forControlEvents:UIControlEventTouchUpInside];
  UIStackView* stack = [[UIStackView alloc] initWithArrangedSubviews:@[ title, _status, _copy, cancel ]];
  stack.axis = UILayoutConstraintAxisVertical;
  stack.alignment = UIStackViewAlignmentCenter;
  stack.spacing = 16;
  stack.translatesAutoresizingMaskIntoConstraints = NO;
  [self.view addSubview:stack];
  [NSLayoutConstraint activateConstraints:@[
    [stack.centerYAnchor constraintEqualToAnchor:self.view.centerYAnchor],
    [stack.leadingAnchor constraintEqualToAnchor:self.view.layoutMarginsGuide.leadingAnchor],
    [stack.trailingAnchor constraintEqualToAnchor:self.view.layoutMarginsGuide.trailingAnchor],
  ]];
  [self readItems];
}

/// Collects every shared text and link (titles, content text, URL and text attachments),
/// then finds the point (found:items:).
- (void)readItems {
  NSMutableArray<NSString*>* texts = [NSMutableArray array];
  NSMutableArray<NSString*>* kinds = [NSMutableArray array];
  dispatch_group_t group = dispatch_group_create();
  const auto add = [texts, kinds](NSString* kind, NSString* text) {
    if (text.length == 0) return;
    @synchronized(texts) {
      [texts addObject:text];
      [kinds addObject:[NSString stringWithFormat:@"%@: %@", kind, text]];
    }
  };
  for (NSExtensionItem* item in self.extensionContext.inputItems) {
    add(@"title", item.attributedTitle.string);
    add(@"content", item.attributedContentText.string);
    for (NSItemProvider* p in item.attachments) {
      NSString* type = [p hasItemConformingToTypeIdentifier:UTTypeURL.identifier]         ? UTTypeURL.identifier
                       : [p hasItemConformingToTypeIdentifier:UTTypePlainText.identifier] ? UTTypePlainText.identifier
                                                                                         : nil;
      if (!type) continue;
      dispatch_group_enter(group);
      [p loadItemForTypeIdentifier:type
                           options:nil
                 completionHandler:^(id<NSSecureCoding> value, NSError*) {
                   if ([(id)value isKindOfClass:NSURL.class]) add(@"url", [(NSURL*)value absoluteString]);
                   else if ([(id)value isKindOfClass:NSString.class]) add(@"text", (NSString*)value);
                   dispatch_group_leave(group);
                 }];
    }
  }
  __weak VrstShareController* weak = self;
  dispatch_group_notify(group, dispatch_get_main_queue(), ^{
    [weak found:texts items:[kinds componentsJoinedByString:@" | "]];
  });
}

/// Picks what to show: the first text with a point; else a short link opened to its
/// target; else the first link or text (the app searches it).
/// @param texts Shared texts and links.
/// @param items Description of all of them, for the app's log.
- (void)found:(NSArray<NSString*>*)texts items:(NSString*)items {
  for (NSString* t in texts)
    if (nav::parse_geo_link(t.UTF8String ?: "")) {
      [self openApp:t items:items];
      return;
    }
  for (NSString* t in texts)
    if (nav::is_short_map_link(t.UTF8String ?: "")) {
      [self resolve:t items:items];
      return;
    }
  if (texts.count == 0) {
    _status.text = @"Nothing to show: no link or text was shared.";
    return;
  }
  [self openApp:texts.firstObject items:items];
}

/// Opens a short link over the internet; the link it redirects to is what is shown.
/// @param link Short link.
/// @param items Description of the shared items.
- (void)resolve:(NSString*)link items:(NSString*)items {
  _status.text = @"Opening the Google Maps link...";
  NSURL* url = [NSURL URLWithString:[link stringByTrimmingCharactersInSet:NSCharacterSet.whitespaceAndNewlineCharacterSet]];
  if (!url) {
    [self openApp:link items:items];
    return;
  }
  __weak VrstShareController* weak = self;
  [[NSURLSession.sharedSession dataTaskWithURL:url
                             completionHandler:^(NSData*, NSURLResponse* response, NSError* error) {
                               NSString* target = response.URL.absoluteString;
                               dispatch_async(dispatch_get_main_queue(), ^{
                                 NSString* more = [items stringByAppendingFormat:@" | opened: %@",
                                                                                  target ?: error.localizedDescription];
                                 [weak openApp:target.length ? target : link items:more];
                               });
                             }] resume];
}

/// Opens the app with the text to show; when that is not possible, offers Copy.
/// @param text What the app shows (coordinates, a link or an address).
/// @param items Description of the shared items, for the app's log.
- (void)openApp:(NSString*)text items:(NSString*)items {
  _result = text;
  NSURLComponents* c = [NSURLComponents componentsWithString:@"verst://share"];
  NSString* shortItems = items.length > kMaxItemsText ? [items substringToIndex:kMaxItemsText] : items;
  c.queryItems = @[ [NSURLQueryItem queryItemWithName:@"text" value:text],
                    [NSURLQueryItem queryItemWithName:@"items" value:shortItems] ];
  if (const auto link = nav::parse_geo_link(text.UTF8String ?: "")) {
    _status.text = [NSString stringWithFormat:@"%@\n%.6f, %.6f", link->name.empty() ? @"Point" : @(link->name.c_str()),
                                              link->pos.lat, link->pos.lon];
  } else {
    _status.text = @"Searching it in Verst...";
  }
  if (![self openURL:c.URL]) {
    _status.text = [_status.text stringByAppendingString:@"\n\nVerst could not be opened from here."];
    _copy.hidden = NO;
    return;
  }
  __weak VrstShareController* weak = self;
  dispatch_after(dispatch_time(DISPATCH_TIME_NOW, static_cast<int64_t>(kCloseSeconds * NSEC_PER_SEC)),
                 dispatch_get_main_queue(), ^{ [weak.extensionContext completeRequestReturningItems:@[] completionHandler:nil]; });
}

/// Opens a URL through the application object up the responder chain: share extensions
/// have no official way to open their app (fine for an app installed by its developer).
/// @param url URL.
/// @return NO when no responder could open it.
- (BOOL)openURL:(NSURL*)url {
  const SEL open = NSSelectorFromString(@"openURL:options:completionHandler:");
  for (UIResponder* r = self; r; r = r.nextResponder) {
    if ([r isKindOfClass:NSClassFromString(@"UIApplication")] && [r respondsToSelector:open]) {
      using Vrst_open_fn = void (*)(id, SEL, NSURL*, NSDictionary*, id);
      reinterpret_cast<Vrst_open_fn>(objc_msgSend)(r, open, url, @{}, nil);
      return YES;
    }
  }
  return NO;
}

/// Copy: the text goes to the clipboard, the panel closes.
- (void)onCopy {
  UIPasteboard.generalPasteboard.string = _result;
  [self.extensionContext completeRequestReturningItems:@[] completionHandler:nil];
}

/// Cancel: closes the panel.
- (void)onCancel {
  [self.extensionContext cancelRequestWithError:[NSError errorWithDomain:NSCocoaErrorDomain code:NSUserCancelledError
                                                                userInfo:nil]];
}

@end
