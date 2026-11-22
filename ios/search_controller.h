/// @file
/// VrstSearchController: the address search screen. A search field on top, the results
/// below, updated as you type; a tap on a result closes the screen and shows it on the
/// map. A "Clear pin" row removes the pin of the last result.

#import <UIKit/UIKit.h>

NS_ASSUME_NONNULL_BEGIN

/// One result row.
@interface VrstSearchHit : NSObject
/// "12 Rue de la Liberte", "Dijon".
@property(nonatomic, copy) NSString* title;
/// "Dijon 21000 - 1.2 km", "village - 8 km".
@property(nonatomic, copy) NSString* detail;
/// "place", "street" or "address".
@property(nonatomic, copy) NSString* kind;
/// Latitude, degrees.
@property(nonatomic) double lat;
/// Longitude, degrees.
@property(nonatomic) double lon;
/// Zoom that shows it well, ground metres per screen point.
@property(nonatomic) double metresPerPoint;
@end

/// The search screen; see the file comment. Shown inside a navigation controller.
@interface VrstSearchController : UITableViewController <UISearchBarDelegate>

/// Builds the screen.
/// @param query Text to start with (the last search), may be empty.
/// @param hasIndex The shown map has an address index; without one a note says so.
/// @param hasPin A pin is shown: the "Clear pin" row is offered.
/// @return The controller.
- (instancetype)initWithQuery:(NSString*)query hasIndex:(BOOL)hasIndex hasPin:(BOOL)hasPin;

/// Runs a search; called 150 ms after the text stops changing.
@property(nonatomic, copy, nullable) NSArray<VrstSearchHit*>* (^searchHandler)(NSString* text);

/// Called when the user picks a result; the screen closes itself first.
@property(nonatomic, copy, nullable) void (^pickHandler)(VrstSearchHit* hit);

/// Called when the user clears the pin; the screen closes itself first.
@property(nonatomic, copy, nullable) void (^clearPinHandler)(void);

/// The text in the search field now.
@property(nonatomic, readonly, copy) NSString* query;

@end

NS_ASSUME_NONNULL_END
