use defmt::Format;
use usbd_hid::descriptor::KeyboardReport;

pub(crate) enum Usage {
    Keyboard(KeyboardUsage),
}

/// KeyboardUsage describes the key codes to be used in implementing a USB keyboard.
///
/// The usage type of all key codes is Selectors, except for the modifier keys
/// Keyboard Left Control to Keyboard Right GUI which are Dynamic Flags.
///
/// Reference: <https://usb.org/sites/default/files/hut1_3_0.pdf> (Section 10, page 88)
#[repr(u8)]
#[allow(unused)]
#[non_exhaustive]
#[derive(Copy, Debug, Clone, Eq, PartialEq, Format)]
pub(crate) enum KeyboardUsage {
    // 0x00: Reserved
    /// Keyboard ErrorRollOver (Footnote 1)
    KeyboardErrorRollOver = 0x01,
    /// Keyboard POSTFail (Footnote 1)
    KeyboardPOSTFail = 0x02,
    /// Keyboard ErrorUndefined (Footnote 1)
    KeyboardErrorUndefined = 0x03,
    /// Keyboard a and A (Footnote 2)
    KeyboardAa = 0x04,
    /// Keyboard b and B
    KeyboardBb = 0x05,
    /// Keyboard c and C (Footnote 2)
    KeyboardCc = 0x06,
    /// Keyboard d and D
    KeyboardDd = 0x07,
    /// Keyboard e and E
    KeyboardEe = 0x08,
    /// Keyboard f and F
    KeyboardFf = 0x09,
    /// Keyboard g and G
    KeyboardGg = 0x0A,
    /// Keyboard h and H
    KeyboardHh = 0x0B,
    /// Keyboard i and I
    KeyboardIi = 0x0C,
    /// Keyboard j and J
    KeyboardJj = 0x0D,
    /// Keyboard k and K
    KeyboardKk = 0x0E,
    /// Keyboard l and L
    KeyboardLl = 0x0F,
    /// Keyboard m and M (Footnote 2)
    KeyboardMm = 0x10,
    /// Keyboard n and N
    KeyboardNn = 0x11,
    /// Keyboard o and O (Footnote 2)
    KeyboardOo = 0x12,
    /// Keyboard p and P (Footnote 2)
    KeyboardPp = 0x13,
    /// Keyboard q and Q (Footnote 2)
    KeyboardQq = 0x14,
    /// Keyboard r and R
    KeyboardRr = 0x15,
    /// Keyboard s and S
    KeyboardSs = 0x16,
    /// Keyboard t and T
    KeyboardTt = 0x17,
    /// Keyboard u and U
    KeyboardUu = 0x18,
    /// Keyboard v and V
    KeyboardVv = 0x19,
    /// Keyboard w and W (Footnote 2)
    KeyboardWw = 0x1A,
    /// Keyboard x and X (Footnote 2)
    KeyboardXx = 0x1B,
    /// Keyboard y and Y (Footnote 2)
    KeyboardYy = 0x1C,
    /// Keyboard z and Z (Footnote 2)
    KeyboardZz = 0x1D,
    /// Keyboard 1 and ! (Footnote 2)
    Keyboard1Exclamation = 0x1E,
    /// Keyboard 2 and @ (Footnote 2)
    Keyboard2At = 0x1F,
    /// Keyboard 3 and # (Footnote 2)
    Keyboard3Hash = 0x20,
    /// Keyboard 4 and $ (Footnote 2)
    Keyboard4Dollar = 0x21,
    /// Keyboard 5 and % (Footnote 2)
    Keyboard5Percent = 0x22,
    /// Keyboard 6 and ^ (Footnote 2)
    Keyboard6Caret = 0x23,
    /// Keyboard 7 and & (Footnote 2)
    Keyboard7Ampersand = 0x24,
    /// Keyboard 8 and * (Footnote 2)
    Keyboard8Asterisk = 0x25,
    /// Keyboard 9 and ( (Footnote 2)
    Keyboard9OpenParens = 0x26,
    /// Keyboard 0 and ) (Footnote 2)
    Keyboard0CloseParens = 0x27,
    /// Keyboard Return (ENTER) (Footnote 3)
    ///
    ///  (Footnote 3): Keyboard Enter and Keypad Enter generate different Usage codes.
    KeyboardEnter = 0x28,
    /// Keyboard ESCAPE
    KeyboardEscape = 0x29,
    /// Keyboard DELETE (Backspace) (Footnote 4)
    KeyboardBackspace = 0x2A,
    /// Keyboard Tab
    KeyboardTab = 0x2B,
    /// Keyboard Spacebar
    KeyboardSpacebar = 0x2C,
    /// Keyboard - and _ (Footnote 2)
    KeyboardDashUnderscore = 0x2D,
    /// Keyboard = and + (Footnote 2)
    KeyboardEqualPlus = 0x2E,
    /// Keyboard [ and { (Footnote 2)
    KeyboardOpenBracketBrace = 0x2F,
    /// Keyboard ] and } (Footnote 2)
    KeyboardCloseBracketBrace = 0x30,
    /// Keyboard \ and |
    KeyboardBackslashBar = 0x31,
    /// Keyboard Non-US # and (Footnote 5)
    KeyboardNonUSHash = 0x32,
    /// Keyboard ; and : (Footnote 2)
    KeyboardSemiColon = 0x33,
    /// Keyboard ' and " (Footnote 2)
    KeyboardSingleDoubleQuote = 0x34,
    /// Keyboard ` and ~ (Footnote 2)
    KeyboardBacktickTilde = 0x35,
    /// Keyboard , and < (Footnote 2)
    KeyboardCommaLess = 0x36,
    /// Keyboard . and > (Footnote 2)
    KeyboardPeriodGreater = 0x37,
    /// Keyboard / and ? (Footnote 2)
    KeyboardSlashQuestion = 0x38,
    /// Keyboard Caps Lock (Footnote 6)
    KeyboardCapsLock = 0x39,
    /// Keyboard F1
    KeyboardF1 = 0x3A,
    /// Keyboard F2
    KeyboardF2 = 0x3B,
    /// Keyboard F3
    KeyboardF3 = 0x3C,
    /// Keyboard F4
    KeyboardF4 = 0x3D,
    /// Keyboard F5
    KeyboardF5 = 0x3E,
    /// Keyboard F6
    KeyboardF6 = 0x3F,
    /// Keyboard F7
    KeyboardF7 = 0x40,
    /// Keyboard F8
    KeyboardF8 = 0x41,
    /// Keyboard F9
    KeyboardF9 = 0x42,
    /// Keyboard F10
    KeyboardF10 = 0x43,
    /// Keyboard F11
    KeyboardF11 = 0x44,
    /// Keyboard F12
    KeyboardF12 = 0x45,
    /// Keyboard PrintScreen (Footnote 7)
    KeyboardPrintScreen = 0x46,
    /// Keyboard ScrollLock (Footnote 6)
    KeyboardScrollLock = 0x47,
    /// Keyboard Pause (Footnote 7)
    KeyboardPause = 0x48,
    /// Keyboard Insert (Footnote 7)
    KeyboardInsert = 0x49,
    /// Keyboard Home (Footnote 7)
    KeyboardHome = 0x4A,
    /// Keyboard PageUp (Footnote 7)
    KeyboardPageUp = 0x4B,
    /// Keyboard Delete Forward (Footnote 7) (Footnote 8)
    KeyboardDelete = 0x4C,
    /// Keyboard End (Footnote 7)
    KeyboardEnd = 0x4D,
    /// Keyboard PageDown (Footnote 7)
    KeyboardPageDown = 0x4E,
    /// Keyboard RightArrow (Footnote 7)
    KeyboardRightArrow = 0x4F,
    /// Keyboard LeftArrow (Footnote 7)
    KeyboardLeftArrow = 0x50,
    /// Keyboard DownArrow (Footnote 7)
    KeyboardDownArrow = 0x51,
    /// Keyboard UpArrow (Footnote 7)
    KeyboardUpArrow = 0x52,
    /// Keypad Num Lock and Clear (Footnote 6)
    KeypadNumLock = 0x53,
    /// Keypad / (Footnote 7)
    KeypadDivide = 0x54,
    /// Keypad *
    KeypadMultiply = 0x55,
    /// Keypad -
    KeypadMinus = 0x56,
    /// Keypad +
    KeypadPlus = 0x57,
    /// Keypad ENTER (Footnote 3)
    KeypadEnter = 0x58,
    /// Keypad 1 and End
    Keypad1End = 0x59,
    /// Keypad 2 and DownArrow
    Keypad2DownArrow = 0x5A,
    /// Keypad 3 and PageDown
    Keypad3PageDown = 0x5B,
    /// Keypad 4 and LeftArrow
    Keypad4LeftArrow = 0x5C,
    /// Keypad 5
    Keypad5 = 0x5D,
    /// Keypad 6 and RightArrow
    Keypad6RightArrow = 0x5E,
    /// Keypad 7 and Home
    Keypad7Home = 0x5F,
    /// Keypad 8 and UpArrow
    Keypad8UpArrow = 0x60,
    /// Keypad 9 and PageUp
    Keypad9PageUp = 0x61,
    /// Keypad 0 and Insert
    Keypad0Insert = 0x62,
    /// Keypad . and Delete
    KeypadPeriodDelete = 0x63,
    /// Keyboard Non-US \ and | (Footnote 9) (Footnote 10)
    KeyboardNonUSSlash = 0x64,
    /// Keyboard Application (Footnote 11)
    KeyboardApplication = 0x65,
    /// Keyboard Power (Footnote 1)
    KeyboardPower = 0x66,
    /// Keypad =
    KeypadEqual = 0x67,
    /// Keyboard F13
    KeyboardF13 = 0x68,
    /// Keyboard F14
    KeyboardF14 = 0x69,
    /// Keyboard F15
    KeyboardF15 = 0x6A,
    /// Keyboard F16
    KeyboardF16 = 0x6B,
    /// Keyboard F17
    KeyboardF17 = 0x6C,
    /// Keyboard F18
    KeyboardF18 = 0x6D,
    /// Keyboard F19
    KeyboardF19 = 0x6E,
    /// Keyboard F20
    KeyboardF20 = 0x6F,
    /// Keyboard F21
    KeyboardF21 = 0x70,
    /// Keyboard F22
    KeyboardF22 = 0x71,
    /// Keyboard F23
    KeyboardF23 = 0x72,
    /// Keyboard F24
    KeyboardF24 = 0x73,
    /// Keyboard Execute
    KeyboardExecute = 0x74,
    /// Keyboard Help
    KeyboardHelp = 0x75,
    /// Keyboard Menu
    KeyboardMenu = 0x76,
    /// Keyboard Select
    KeyboardSelect = 0x77,
    /// Keyboard Stop
    KeyboardStop = 0x78,
    /// Keyboard Again
    KeyboardAgain = 0x79,
    /// Keyboard Undo
    KeyboardUndo = 0x7A,
    /// Keyboard Cut
    KeyboardCut = 0x7B,
    /// Keyboard Copy
    KeyboardCopy = 0x7C,
    /// Keyboard Paste
    KeyboardPaste = 0x7D,
    /// Keyboard Find
    KeyboardFind = 0x7E,
    /// Keyboard Mute
    KeyboardMute = 0x7F,
    /// Keyboard Volume Up
    KeyboardVolumeUp = 0x80,
    /// Keyboard Volume Down
    KeyboardVolumeDown = 0x81,
    /// Keyboad Locking Caps Lock (Footnote 12)
    KeyboardLockingCapsLock = 0x82,
    /// Keyboad Locking Num Lock (Footnote 12)
    KeyboardLockingNumLock = 0x83,
    /// Keyboad Locking Scroll Lock (Footnote 12)
    KeyboardLockingScrollLock = 0x84,
    /// Keypad Comma (Footnote 13)
    KeypadComma = 0x85,
    /// Keypad Equal Sign (Footnote 14)
    KeypadEqualSign = 0x86,
    /// Keyboard International1 (Footnote 15) (Footnote 16)
    KeyboardInternational1 = 0x87,
    /// Keyboard International2 (Footnote 17)
    KeyboardInternational2 = 0x88,
    /// Keyboard International3 (Footnote 18)
    KeyboardInternational3 = 0x89,
    /// Keyboard International4 (Footnote 19)
    KeyboardInternational4 = 0x8A,
    /// Keyboard International5 (Footnote 20)
    KeyboardInternational5 = 0x8B,
    /// Keyboard International6 (Footnote 21)
    KeyboardInternational6 = 0x8C,
    /// Keyboard International7 (Footnote 22)
    KeyboardInternational7 = 0x8D,
    /// Keyboard International8 (Footnote 23)
    KeyboardInternational8 = 0x8E,
    /// Keyboard International9 (Footnote 23)
    KeyboardInternational9 = 0x8F,
    /// Keyboard LANG1 (Footnote 24)
    KeyboardLANG1 = 0x90,
    /// Keyboard LANG2 (Footnote 25)
    KeyboardLANG2 = 0x91,
    /// Keyboard LANG3 (Footnote 26)
    KeyboardLANG3 = 0x92,
    /// Keyboard LANG4 (Footnote 27)
    KeyboardLANG4 = 0x93,
    /// Keyboard LANG5 (Footnote 28)
    KeyboardLANG5 = 0x94,
    /// Keyboard LANG6 (Footnote 29)
    KeyboardLANG6 = 0x95,
    /// Keyboard LANG7 (Footnote 29)
    KeyboardLANG7 = 0x96,
    /// Keyboard LANG8 (Footnote 29)
    KeyboardLANG8 = 0x97,
    /// Keyboard LANG9 (Footnote 29)
    KeyboardLANG9 = 0x98,
    /// Keyboard Alternate Erase (Footnote 30)
    KeyboardAlternateErase = 0x99,
    /// Keyboard SysReq/Attention (Footnote 7)
    KeyboardSysReqAttention = 0x9A,
    /// Keyboard Cancel
    KeyboardCancel = 0x9B,
    /// Keyboard Clear
    KeyboardClear = 0x9C,
    /// Keyboard Prior
    KeyboardPrior = 0x9D,
    /// Keyboard Return
    KeyboardReturn = 0x9E,
    /// Keyboard Separator
    KeyboardSeparator = 0x9F,
    /// Keyboard Out
    KeyboardOut = 0xA0,
    /// Keyboard Oper
    KeyboardOper = 0xA1,
    /// Keyboard Clear/Again
    KeyboardClearAgain = 0xA2,
    /// Keyboard CrSel/Props
    KeyboardCrSelProps = 0xA3,
    /// Keyboard ExSel
    KeyboardExSel = 0xA4,
    // 0xA5-0xAF: Reserved
    /// Keypad 00
    Keypad00 = 0xB0,
    /// Keypad 000
    Keypad000 = 0xB1,
    /// Thousands Separator (Footnote 31)
    ThousandsSeparator = 0xB2,
    /// Decimal Separator (Footnote 31)
    DecimalSeparator = 0xB3,
    /// Currency Unit (Footnote 32)
    CurrencyUnit = 0xB4,
    /// Currency Sub-unit (Footnote 32)
    CurrencySubunit = 0xB5,
    /// Keypad (
    KeypadOpenParens = 0xB6,
    /// Keypad )
    KeypadCloseParens = 0xB7,
    /// Keypad {
    KeypadOpenBrace = 0xB8,
    /// Keypad }
    KeypadCloseBrace = 0xB9,
    /// Keypad Tab
    KeypadTab = 0xBA,
    /// Keypad Backspace
    KeypadBackspace = 0xBB,
    /// Keypad A
    KeypadA = 0xBC,
    /// Keypad B
    KeypadB = 0xBD,
    /// Keypad C
    KeypadC = 0xBE,
    /// Keypad D
    KeypadD = 0xBF,
    /// Keypad E
    KeypadE = 0xC0,
    /// Keypad F
    KeypadF = 0xC1,
    /// Keypad XOR
    KeypadBitwiseXor = 0xC2,
    /// Keypad ^
    KeypadLogicalXor = 0xC3,
    /// Keypad %
    KeypadModulo = 0xC4,
    /// Keypad <
    KeypadLeftShift = 0xC5,
    /// Keypad >
    KeypadRightShift = 0xC6,
    /// Keypad &
    KeypadBitwiseAnd = 0xC7,
    /// Keypad &&
    KeypadLogicalAnd = 0xC8,
    /// Keypad |
    KeypadBitwiseOr = 0xC9,
    /// Keypad ||
    KeypadLogicalOr = 0xCA,
    /// Keypad :
    KeypadColon = 0xCB,
    /// Keypad #
    KeypadHash = 0xCC,
    /// Keypad Space
    KeypadSpace = 0xCD,
    /// Keypad @
    KeypadAt = 0xCE,
    /// Keypad !
    KeypadExclamation = 0xCF,
    /// Keypad Memory Store
    KeypadMemoryStore = 0xD0,
    /// Keypad Memory Recall
    KeypadMemoryRecall = 0xD1,
    /// Keypad Memory Clear
    KeypadMemoryClear = 0xD2,
    /// Keypad Memory Add
    KeypadMemoryAdd = 0xD3,
    /// Keypad Memory Subtract
    KeypadMemorySubtract = 0xD4,
    /// Keypad Memory Multiply
    KeypadMemoryMultiply = 0xD5,
    /// Keypad Memory Divice
    KeypadMemoryDivide = 0xD6,
    /// Keypad +/-
    KeypadPositiveNegative = 0xD7,
    /// Keypad Clear
    KeypadClear = 0xD8,
    /// Keypad Clear Entry
    KeypadClearEntry = 0xD9,
    /// Keypad Binary
    KeypadBinary = 0xDA,
    /// Keypad Octal
    KeypadOctal = 0xDB,
    /// Keypad Decimal
    KeypadDecimal = 0xDC,
    /// Keypad Hexadecimal
    KeypadHexadecimal = 0xDD,
    // 0xDE-0xDF: Reserved
    /// Keyboard LeftControl
    KeyboardLeftControl = 0xE0,
    /// Keyboard LeftShift
    KeyboardLeftShift = 0xE1,
    /// Keyboard LeftAlt
    KeyboardLeftAlt = 0xE2,
    /// Keyboard LeftGUI (Footnote 11) (Footnote 33)
    KeyboardLeftGUI = 0xE3,
    /// Keyboard RightControl
    KeyboardRightControl = 0xE4,
    /// Keyboard RightShift
    KeyboardRightShift = 0xE5,
    /// Keyboard RightAlt
    KeyboardRightAlt = 0xE6,
    /// Keyboard RightGUI (Footnote 11) (Footnote 34)
    KeyboardRightGUI = 0xE7,
    /// Reserved keyboard values (used for all reserved / invalid values)
    Reserved = 0xE8,
    // 0xE8-0xFF: Reserved
}

impl KeyboardUsage {
    fn is_dynamic_flag(self) -> bool {
        (Self::KeyboardLeftControl as u8..=Self::KeyboardRightGUI as u8).contains(&(self as u8))
    }

    pub fn as_modifier_mask(self) -> u8 {
        if self.is_dynamic_flag() {
            1 << (self as u8 - Self::KeyboardLeftControl as u8)
        } else {
            0
        }
    }
}

pub(crate) fn key_to_usage(key: u16) -> Option<Usage> {
    'keyboard: {
        return Some(Usage::Keyboard(match key {
            // 0 unused
            1 => KeyboardUsage::KeypadDivide,
            2 => KeyboardUsage::KeypadMultiply,
            3 => KeyboardUsage::KeypadEqual,
            4 => KeyboardUsage::Keypad7Home,
            5 => KeyboardUsage::Keypad8UpArrow,
            6 => KeyboardUsage::Keypad9PageUp,
            7 => KeyboardUsage::KeypadMinus,
            8 => KeyboardUsage::Keypad4LeftArrow,
            9 => KeyboardUsage::Keypad5,
            10 => KeyboardUsage::Keypad6RightArrow,
            11 => KeyboardUsage::KeypadPlus,
            12 => KeyboardUsage::Keypad1End,
            13 => KeyboardUsage::Keypad2DownArrow,
            14 => KeyboardUsage::Keypad3PageDown,
            15 => KeyboardUsage::KeypadEnter,
            16 => KeyboardUsage::Keypad0Insert,
            17 => KeyboardUsage::Keypad0Insert,
            18 => KeyboardUsage::KeypadPeriodDelete,
            19 => KeyboardUsage::KeypadEnter,
            _ => break 'keyboard,
        }));
    };
    None
}

pub(crate) fn register_usage(r: &mut KeyboardReport, key: Usage) {
    match key {
        Usage::Keyboard(key) => {
            if key.is_dynamic_flag() {
                register_modifier(r, key.as_modifier_mask());
            } else {
                register_keycode(r, key);
            }
        }
    }
}

pub(crate) fn unregister_usage(r: &mut KeyboardReport, key: Usage) {
    match key {
        Usage::Keyboard(key) => {
            if key.is_dynamic_flag() {
                unregister_modifier(r, key.as_modifier_mask());
            } else {
                unregister_keycode(r, key);
            }
        }
    }
}

fn register_keycode(r: &mut KeyboardReport, key: KeyboardUsage) {
    if let Some(k) = r.keycodes.iter_mut().find(|k| **k == 0) {
        *k = key as u8;
    }
}

fn unregister_keycode(r: &mut KeyboardReport, key: KeyboardUsage) {
    if let Some(k) = r.keycodes.iter_mut().find(|k| **k == key as u8) {
        *k = 0;
    }
}

fn register_modifier(r: &mut KeyboardReport, modifier_mask: u8) {
    r.modifier |= modifier_mask;
}

fn unregister_modifier(r: &mut KeyboardReport, modifier_mask: u8) {
    r.modifier &= !modifier_mask;
}
