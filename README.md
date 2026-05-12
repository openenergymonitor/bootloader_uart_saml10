# ATSAML10 Serial Bootloader

This is a serial bootloader for [Microchip (née Atmel) ATSAML11 microcontrollers](https://www.microchip.com/en-us/products/microcontrollers-and-microprocessors/32-bit-mcus/sam-32-bit-mcus/sam-l/sam-l10-l11). It is an updated version of [Microhip's bootloader implementation](https://www.microchip.com/en-us/application-notes/an2699).

- Uses Python 3 rather than Python 2.
- Self hosted, doesn't require abandoned Atmel IDE.

## Bootloader Usage

### Entering the bootloader

The bootloader will be entered when either of the following conditions are met:

- There is no application uploaded
- The BOOT button is pressed at reset

The LED will flash at ~5 Hz to indicate it is in the bootloader.

### Using the bootloader

> [!NOTE]
> The steps below are for Linux/macOS, I do not have a Windows machine to test this at the moment.

### Setting up the environment

The uploader is `./tools/boot.py`. A virtual environment should be set up before first use. In `./tools/` run:

```shell
python3 -m venv venv && source venv/bin/activate && pip install -r requirements.txt
```

> [!NOTE]
> This assumes you are using `bash` or `zsh`. For `fish` and `tcsh` append `.fish` and `.csh` respectively to `source venv/bin/activate`.

### Uploading your code

Before you start, activate the virtual environment. In `./tools/` run:

```shell
source venv/bin/activate
```

Now, you can upload your firmware to the microcontroller. In `./tools/` run:

```shell
python3 boot.py -v -i SERIAL_PORT -f PATH_TO_BIN -o 0x400
python3 boot.py -v -i SERIAL_PORT -r
```

This first loads the firmware and then resets the device. If the BOOT pin is not pulled LOW, the SAML10 will enter the application.

The error `Error: Unknown response byte: 0xbf` means three's an issue with the UART connection or the device is not in bootloader mode

If you need to list your available `SERIAL_PORT`s, simply run `python3 boot.py`.

### Troubleshooting

- Is the bootloader responsive?
  - Open the serial device in a terminal. 115200 baud, 8+1 bits, no parity.
  - Type any key. The bootloader should return `Q` in return.
- Check the Python version you are using, it must be Python 3.
- Is the virtual environment activated?

## Building the bootloader

### Altering

You need to alter the pin configuration to match your board's bootloader entry pin and optionally LED indication and regulator enable. This can be done by setting the following `#define`s in `src/saml_bl.c`:

- `BL_REQ_PIN`: pin for bootloader entry button/jumper.
- `BL_LED_PIN`: pin for LED (active high or low).
- `BL_REG_PIN`: pin for regulator enable.

The following `#define`s select the LED and regulator enable functionality:

- `BL_LED_EN`: set to `1` to enable LED indication, `0` to disable.
- `BL_REG_EN`: set to `1` to enable regulator enable, `0` to disable.
- `BL_REG_HIGH`: set to `1` for active HIGH regulator enable, `0` for active LOW.

### Building

To build the bootloader, you need the [Arm Embedded Toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads) installed. With this on your path, simply run:

> make

This will produce `build/saml10_bl.bin` which you can upload to the microcontroller as described above.
