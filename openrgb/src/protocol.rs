type RGBColor = u32;

pub trait RGBColorMethods {
    fn RGBGetRValue(&self) -> RGBColor;
    fn RGBGetGValue(&self) -> RGBColor;
    fn RGBGetBValue(&self) -> RGBColor;
}

impl RGBColorMethods for RGBColor {
    fn RGBGetRValue(&self) -> RGBColor {
        self & 0x000000FF
    }
    fn RGBGetGValue(&self) -> RGBColor {
        (self >> 8) & 0x000000FF
    }
    fn RGBGetBValue(&self) -> RGBColor {
        (self >> 16) & 0x000000FF
    }
}

pub fn ToRGBColor(r: u8, g: u8, b: u8) -> RGBColor {
    u32::from_be_bytes([0, r, g, b])
}

/*------------------------------------------------------------------*\
| Mode Flags                                                         |
\*------------------------------------------------------------------*/
pub enum RGBModeFlag {
    MODE_FLAG_HAS_SPEED = (1 << 0), /* Mode has speed parameter         */
    MODE_FLAG_HAS_DIRECTION_LR = (1 << 1), /* Mode has left/right parameter    */
    MODE_FLAG_HAS_DIRECTION_UD = (1 << 2), /* Mode has up/down parameter       */
    MODE_FLAG_HAS_DIRECTION_HV = (1 << 3), /* Mode has horiz/vert parameter    */
    MODE_FLAG_HAS_BRIGHTNESS = (1 << 4), /* Mode has brightness parameter    */
    MODE_FLAG_HAS_PER_LED_COLOR = (1 << 5), /* Mode has per-LED colors          */
    MODE_FLAG_HAS_MODE_SPECIFIC_COLOR = (1 << 6), /* Mode has mode specific colors    */
    MODE_FLAG_HAS_RANDOM_COLOR = (1 << 7), /* Mode has random color option     */
    MODE_FLAG_MANUAL_SAVE = (1 << 8), /* Mode can manually be saved       */
    MODE_FLAG_AUTOMATIC_SAVE = (1 << 9), /* Mode automatically saves         */
}

/*------------------------------------------------------------------*\
| Mode Directions                                                    |
\*------------------------------------------------------------------*/
pub enum RGBModeDirection {
    MODE_DIRECTION_LEFT = 0,       /* Mode direction left              */
    MODE_DIRECTION_RIGHT = 1,      /* Mode direction right             */
    MODE_DIRECTION_UP = 2,         /* Mode direction up                */
    MODE_DIRECTION_DOWN = 3,       /* Mode direction down              */
    MODE_DIRECTION_HORIZONTAL = 4, /* Mode direction horizontal        */
    MODE_DIRECTION_VERTICAL = 5,   /* Mode direction vertical          */
}

/*------------------------------------------------------------------*\
| Mode Color Types                                                   |
\*------------------------------------------------------------------*/
pub enum RGBModeColorType {
    MODE_COLORS_NONE = 0,          /* Mode has no colors               */
    MODE_COLORS_PER_LED = 1,       /* Mode has per LED colors selected */
    MODE_COLORS_MODE_SPECIFIC = 2, /* Mode specific colors selected    */
    MODE_COLORS_RANDOM = 3,        /* Mode has random colors selected  */
}

/*------------------------------------------------------------------*\
| Mode Class                                                         |
\*------------------------------------------------------------------*/
pub struct RGBMode {
    /*--------------------------------------------------------------*\
    | Mode Information                                               |
    \*--------------------------------------------------------------*/
    name: String,        /* Mode name                        */
    value: i32,          /* Device-specific mode value       */
    flags: u32,          /* Mode flags bitfield              */
    speed_min: u32,      /* speed minimum value          */
    speed_max: u32,      /* speed maximum value          */
    brightness_min: u32, /*brightness min value      */
    brightness_max: u32, /*brightness max value      */
    colors_min: u32,     /* minimum number of mode colors*/
    colors_max: u32,     /* maximum numver of mode colors*/

    /*--------------------------------------------------------------*\
    | Mode Settings                                                  |
    \*--------------------------------------------------------------*/
    speed: u32,            /* Mode speed parameter value       */
    brightness: u32,       /* Mode brightness value        */
    direction: u32,        /* Mode direction value         */
    color_mode: u32,       /* Mode color selection         */
    colors: Vec<RGBColor>, /* mode-specific colors             */
}

/*------------------------------------------------------------------*\
| LED Struct                                                         |
\*------------------------------------------------------------------*/
pub struct RGBLED {
    name: String, /* LED name                     */
    value: u32,   /* Device-specific LED value    */
}

/*------------------------------------------------------------------*\
| Zone Types                                                         |
\*------------------------------------------------------------------*/
pub enum RGBZoneType {
    ZONE_TYPE_SINGLE,
    ZONE_TYPE_LINEAR,
    ZONE_TYPE_MATRIX,
}

/*------------------------------------------------------------------*\
| Matrix Map Struct                                                  |
\*------------------------------------------------------------------*/
pub struct RGBMatrixMapType {
    height: u32,
    width: u32,
    map: u32, // is actually pointer to map...?
}

/*------------------------------------------------------------------*\
| Segment Struct                                                     |
\*------------------------------------------------------------------*/
pub struct RGBSegment {
    name: String,            /* Segment name             */
    type_field: RGBZoneType, /* Segment type             */
    start_idx: u32,          /* Start index within zone  */
    leds_count: u32,         /* Number of LEDs in segment*/
}

/*------------------------------------------------------------------*\
| Zone Struct                                                        |
\*------------------------------------------------------------------*/
pub struct RGBZone<'a> {
    name: String,                     /* Zone name                */
    type_field: RGBZoneType,          /* Zone type                */
    leds: Vec<RGBLED>,                /* List of LEDs in zone     */
    colors: Vec<RGBColor>,            /* Colors of LEDs in zone   */
    start_idx: u32,                   /* Start index of led/color */
    leds_count: u32,                  /* Number of LEDs in zone   */
    leds_min: u32,                    /* Minimum number of LEDs   */
    leds_max: u32,                    /* Maximum number of LEDs   */
    matrix_map: &'a RGBMatrixMapType, /* Matrix map pointer       */
    segments: Vec<RGBSegment>,        /* Segments in zone         */
}

/*------------------------------------------------------------------*\
| Device Types                                                       |
|   The enum order should be maintained as is for the API however    |
|   DEVICE_TYPE_UNKNOWN needs to remain last. Any new device types   |
|   need to be inserted at the end of the list but before unknown.   |
\*------------------------------------------------------------------*/
pub enum RGBDeviceType {
    DEVICE_TYPE_MOTHERBOARD,
    DEVICE_TYPE_DRAM,
    DEVICE_TYPE_GPU,
    DEVICE_TYPE_COOLER,
    DEVICE_TYPE_LEDSTRIP,
    DEVICE_TYPE_KEYBOARD,
    DEVICE_TYPE_MOUSE,
    DEVICE_TYPE_MOUSEMAT,
    DEVICE_TYPE_HEADSET,
    DEVICE_TYPE_HEADSET_STAND,
    DEVICE_TYPE_GAMEPAD,
    DEVICE_TYPE_LIGHT,
    DEVICE_TYPE_SPEAKER,
    DEVICE_TYPE_VIRTUAL,
    DEVICE_TYPE_STORAGE,
    DEVICE_TYPE_CASE,
    DEVICE_TYPE_MICROPHONE,
    DEVICE_TYPE_ACCESSORY,
    DEVICE_TYPE_KEYPAD,
    DEVICE_TYPE_UNKNOWN,
}

pub trait RGBControllerInterface {
    fn SetupColors(&self);

    fn GetLED(&self, led: u32) -> RGBColor;
    fn SetLED(&self, led: u32, color: RGBColor);
    fn SetAllLEDs(&self, color: RGBColor);
    fn SetAllZoneLEDs(&self, zone: u32, color: RGBColor);

    fn GetMode(&self) -> u32;
    fn SetMode(&self, mode: i32);

    fn GetDeviceDescription(&self, protocol_version: u32) -> String;
    fn ReadDeviceDescription(&self, buf: &mut str, protocol_version: u32);

    fn GetModeDescription(&self, mode: i32, protocol_version: u32) -> String;
    fn SetModeDescription(&self, desc: &str, protocol_version: u32);

    fn GetColorDescription(&self) -> String;
    fn SetColorDescription(&self, desc: &str);

    fn GetZoneColorDescription(&self, zone: i32) -> String;
    fn SetZoneColorDescription(&self, desc: String);

    fn GetSingleLEDColorDescription(&self, led: i32) -> String;
    fn SetSingleLEDColorDescription(&self, desc: String);

    // fn            RegisterUpdateCallback(&self, RGBControllerCallback new_callback, void * new_callback_arg) ;
    // fn            UnregisterUpdateCallback(&self, void * callback_arg)                                       ;
    fn ClearCallbacks(&self);
    fn SignalUpdate(&self);

    fn UpdateLEDs(&self);
    //virtual void          UpdateZoneLEDs(&self, int zone)                                                            ;
    //virtual void          UpdateSingleLED(&self, led: i32)                                                            ;

    fn UpdateMode(&self);
    fn SaveMode(&self);

    fn DeviceCallThreadFunction(&self);

    /*---------------------------------------------------------*\
    | Functions to be implemented in device implementation      |
    \*---------------------------------------------------------*/
    fn SetupZones(&self);

    fn ResizeZone(&self, zone: i32, new_size: i32);

    fn DeviceUpdateLEDs(&self);
    fn UpdateZoneLEDs(&self, zone: i32);
    fn UpdateSingleLED(&self, led: i32);

    fn DeviceUpdateMode(&self);
    fn DeviceSaveMode(&self);

    fn SetCustomMode(&self);
}
