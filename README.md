# MultiWii Serial Protocol ![license](https://img.shields.io/badge/license-MIT-green) ![open source](https://badgen.net/badge/open/source/blue?icon=github)

MultiWii Serial Protocol framework, ported from the Betaflight implementation.

## License Information

This library is a port (modification) of the MultiWii Serial Protocol implementation
in Betaflight (which itself was a port of the Cleanflight implementation).

Both Betaflight and Cleanflight are licensed under the GNU GPL

The original Betaflight copyright notice is included in License.txt and the program files,
as per the GNU GPL "keep intact all notices” requirement.

## Class structure

```mermaid
classDiagram
    class MspSerial {
        virtual int send_frame() = 0
        virtual void process_input() = 0;
    }
    class MspBase {
    }
    class MSP_Stream {
    }
    MSP_Stream *-- MspBase
    MSP_Stream o-- MspSerial
```
