# MultiWii Serial Protocol ![license](https://img.shields.io/badge/license-MIT-green) ![open source](https://badgen.net/badge/open/source/blue?icon=github)

MultiWii Serial Protocol framework, ported from the Betaflight implementation.

## License Information

This library is a port (modification) of the Blackbox implementation
in Betaflight (which itself was a port of the Cleanflight implementation).

Both Betaflight and Cleanflight are licensed under the GNU GPL

The original Betaflight copyright notice is included in License.txt and the program files,
as per the GNU GPL "keep intact all notices” requirement.

## Class structure

```mermaid
classDiagram
    class MSP_SerialBase {
        virtual int sendFrame() = 0
        virtual void processInput() = 0;
    }
    class MSP_Base {
    }
    class MSP_Stream {
    }
    MSP_Stream *-- MSP_Base
    MSP_Stream o-- MSP_SerialBase
```
