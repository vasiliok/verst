/// @file
/// VrstPinCard: the card of a dropped pin, above the toolbar. It names the place (the
/// nearest road, or the search result), shows its coordinates or the route found, lets
/// the user pick the travel mode (Car / Bike / Walk, remembered as "who I am"), and has
/// the route buttons: Route (to this pin), Start here (this pin is the start of the next
/// route), Way back (the saved way back of the last route, or There to switch back) and
/// a close button that clears the pins, the route and the saved way back.

#import <UIKit/UIKit.h>

NS_ASSUME_NONNULL_BEGIN

/// Travel mode of the card's switch; the values match nav::Vrst_travel_mode.
typedef NS_ENUM(NSInteger, VrstCardMode) {
  VrstCardModeCar = 0,   ///< Car driver.
  VrstCardModeBike = 1,  ///< Cyclist.
  VrstCardModeFoot = 2,  ///< Walker.
};

/// The pin card; see the file comment. Hidden until shown with a title.
@interface VrstPinCard : UIView

/// First line: the place ("Rue de la Liberte", "Dropped pin").
@property(nonatomic, copy) NSString* title;
/// Second line: coordinates, "Routing...", "12.4 km - 14 min" or why there is no route.
@property(nonatomic, copy) NSString* detail;
/// The travel mode shown by the switch; set by the user or the controller.
@property(nonatomic) VrstCardMode mode;
/// Whether "Start here" is offered (not for the pin that already is the start).
@property(nonatomic) BOOL startOffered;
/// Whether "Route" and the mode switch are offered (not for a card that only offers the
/// saved way back).
@property(nonatomic) BOOL routeOffered;
/// Whether the way back button is offered (a way back is saved).
@property(nonatomic) BOOL backOffered;
/// Title of the way back button ("Way back", or "There" while the way back is shown).
@property(nonatomic, copy) NSString* backTitle;

/// Route button.
@property(nonatomic, copy, nullable) void (^routeHandler)(void);
/// Start here button.
@property(nonatomic, copy, nullable) void (^startHandler)(void);
/// Way back button.
@property(nonatomic, copy, nullable) void (^backHandler)(void);
/// Close button: clear the pins and the route.
@property(nonatomic, copy, nullable) void (^closeHandler)(void);
/// The user changed the travel mode.
@property(nonatomic, copy, nullable) void (^modeHandler)(VrstCardMode mode);

@end

NS_ASSUME_NONNULL_END
