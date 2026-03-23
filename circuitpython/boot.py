import usb_cdc

# Use a dedicated USB CDC data channel for the host app so REPL text does not
# get mixed into the JSON protocol stream.
usb_cdc.enable(console=False, data=True)
