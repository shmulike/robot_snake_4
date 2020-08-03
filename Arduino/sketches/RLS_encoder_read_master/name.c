#include <usb_names.h>

#define MANUFACTURER_NAME          \
	{                          \
		'M', '4', 'R', '5' \
	}
#define MANUFACTURER_NAME_LEN 4

#define PRODUCT_NAME                                                            \
	{                                                                       \
		'K', 'V', 'M', '-', 'L', 'I', 'N', 'K', ' ', 'H', 'O', 'S', 'T' \
	}
#define PRODUCT_NAME_LEN 13

#define SERIAL_NUMBER                                            \
	{                                                        \
		'K', 'V', 'M', 'L', '_', 'H', '_', '0', '0', '1' \
	}
#define SERIAL_NUMBER_LEN 10

struct usb_string_descriptor_struct usb_string_manufacturer_name = {
	2 + MANUFACTURER_NAME_LEN * 2,
	3,
	MANUFACTURER_NAME};

struct usb_string_descriptor_struct usb_string_product_name = {
	2 + PRODUCT_NAME_LEN * 2,
	3,
	PRODUCT_NAME};

struct usb_string_descriptor_struct usb_string_serial_number = {
	2 + SERIAL_NUMBER_LEN * 2,
	3,
	SERIAL_NUMBER};
