/// @file
/// VrstShareController: the "Verst" entry of the iOS share sheet (share extension).
///
/// Google Maps (or any app) shares a link and often a text with it, for a dropped pin the
/// coordinates ("47 50'29.1"N 4 22'16.2"E"). The extension takes the first of them that
/// holds a point (geo_link.h); a short Google link (maps.app.goo.gl) is opened once over
/// the internet to the link it redirects to. Then it opens the app with
/// verst://share?text=<what to show>&items=<what was shared>, and the app puts the pin
/// there. When the app cannot be opened, the panel shows the text with a Copy button, to
/// paste into the app's search.

#import <UIKit/UIKit.h>

NS_ASSUME_NONNULL_BEGIN

/// The share panel; see the file comment. NSExtensionPrincipalClass of the extension.
@interface VrstShareController : UIViewController
@end

NS_ASSUME_NONNULL_END
