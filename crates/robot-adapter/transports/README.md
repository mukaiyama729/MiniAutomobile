# Transports

Place reusable communication transports here, such as USB serial, CAN, or UDP.
Keep device-specific command encoding in `drivers` and protocol definitions in
the `protocol` submodule. The current Pico USB driver still owns its serial
worker directly; extract shared transport code when another driver needs it.
