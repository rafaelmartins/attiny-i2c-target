# attiny-i2c-device

An implementation of I2C-connected device for Attiny microcontrollers, using USI.

This is useful for projects with multiple MCUs that are already implementing an I2C bus, to offload some logic from the main MCU and/or expand ports.


## How it works?

This library uses the Universal Serial Interface (USI), because most of the Attinys don't implement Two Wire Interface (TWI). As the I2C protocol is very time sensitive, we do not run any operations inside the interruptions that handle the protocol. These interruptions work directly over some virtual registers, that are later used by a timer to actually call the user defined handler function callbacks. For that, the library will use the `Output Compare Match A` interrupt from `Timer0`, and enforce some timer settings, like always run in "Normal Mode" and activete clock without prescaling. The `Output Compare Match B` should be available for usage and setup, but must not be used if the settings enforced by this library are not suitable. Enabling the prescaler after calling `i2c_device_init()` is still possible, if needed.

The library implements an array of virtual registers that can be written and read from I2C master. This array is statically allocated in the data section, and its size can be defined during build time, by defining the `I2C_REGISTER_ALLOC` macro with the desired size as an integer. Users must pay attention to that, because the library will fail silently to add new virtual registers if the array is full. The default size is `10`. The included `Makefile`, that builds the usage example (`test.c`), only implements one virtual register, and will set this macro to `1`.


## How to use it?

Just copy `i2c-device.h` and `i2c-device.c` to your project and add them to your build system. `test.c` is an usage example, as previously mentioned. The [API](#api) section below explains the usage of the functions exported by the library.


## API

The header `i2c-device.h` exports 3 functions:


### `i2c_device_init`

```c
void i2c_device_init(uint8_t addr);
```

This function initializes the USI stack (including ports) and the `Timer0`, enforcing the required timer settings. Users should set other timers before calling this function, and call `sei()` after it, to enable the interruptions.

The only argument of this function is the device address in the I2C bus. Please check the protocol specifications for guidance on how to obtain a reserved address, if your usage requires it.


### `i2c_device_add_register`

```c
void i2c_device_add_register(i2c_device_handler_func_t func);
```

This function adds a new virtual register. The virtual register address is incremental, starting from `0x00` up to the `I2C_REGISTER_ALLOC` size, and defined by the order of addition.

The only argument of this function is a handler function, that will be called by the timer whenever new data arrives in the virtual register. The prototype of the handler function type is:

```c
typedef void (*i2c_device_handler_func_t) (uint8_t reg, uint8_t val);
```

The handler function should accept two arguments: the virtual register address and its current value.


### `i2c_device_set_register`

```c
void i2c_device_set_register(uint8_t reg, uint8_t val);
```

This function sets the value of a virtual register locally. Users should never call the virtual register's handler functions directly, because this will not update the virtual register value itself, then any master read operations to this virtual register will return an erroneous value. Instead, users should call this function to set the virtual registers' value using the timer infrastructure.

The function accepts two arguments: the virtual register address and the value that it should be set to.


## License

This library is released under a 3-clause BSD license. See `LICENSE` file for details.
