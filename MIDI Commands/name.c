/*
   Psalte Organ Works
   Copyright (C) Psalte Organ Works Co. - All Rights Reserved
   Unauthorized copying of this file, via any medium is strictly
   prohibited. This work is proprietary and confidential.
   Written by Ben Lester <Ben@Psalte.com>
*/
// For access by the Arduino build this code must be
// placed into a .c file in the project.  It can not be in
// a .cpp file or the main sketch (the .ino file).
#include "usb_names.h"
// This exact format is required by the USB library.
// The length must match the number of characters.
#define MIDI_NAME   {'P','s','a','l','t','e', ' ', 'M', 'I', 'D', 'I'}
#define MIDI_NAME_LEN  11
struct usb_string_descriptor_struct usb_string_product_name = {
        2 + MIDI_NAME_LEN * 2,
        3,
        MIDI_NAME
};