# Navtech IA SDK

The Navtech IA SDK provides a basic interface to communicate with the IA sensor. The SDK provides the source code for C++ and a .NET DLL that can easily be integrated into applications running on Windows and Linux.


The IA sensor operates in two modes:

* native data - Used to receive the raw fft data from the sensor

* plot extraction - Used to receive only points where an object is present

The SDK provides support for both modes, allowing developers to receive either type of data from the sensor and also provides an interface to send control and configuration messages to the sensor.

Communication between the sensor and the software SDK is over Ethernet and utilises a proprietary binary communication protocol called _Colossus Network Protocol_. The SDK abstracts this protocol to avoid having to develop the low-level socket and messaging processing code. The [Colossus Protocol documentation can be found here](https://navtechradar.atlassian.net/wiki/display/PROD/Colossus+Network+Data+Protocol).

## Directory Overview

### cpp

Contains the old (depreciated) version of the IASDK, built using the c++11 standard. No further changes will be made to this version of the SDK

### cpp_17

Contains the new version of the IASDK, built using the c++17 standard. This folder is configured to use VS Code Build Task and CMake Extensions to enable fast development setup times.

### csharp

Contains a C# version of the IASDK

### iasdk

Contains the IASDK Visual Studio solution files

### protobuf

Contains the protobuf files used in the IASDK

### ros1

Contains the (now depreciated) ROS1 implementation of the IASDK

### ros2

Contains the new ROS2 implementation of the IASDK

## SDK Requirements

### C++ 11

#### C++11 Compiler
* GCC 4.8 and above
* Clang 3.5 and above
* Visual Studio 2019 (VC++ 2019 runtime libraries)

### C++ 17

#### C++17 Compiler
* GCC 9.x and above
* Clang 10 and above

### Microsoft .NET

.NET 4.8 and above

## Linux Specific Requirements

The SDK uses a number of shell scripts as part of its functionality. To use the shell scripts we require bash on Ubuntu. To ensure the necessary shell facilities are available we recommend executing the following command:
```
sudo dpkg-reconfigure -p critical dash
```

## License

See file `LICENSE.txt` or go to <https://opensource.org/licenses/MIT> for full license details.

## Building the C++ SDK
The SDK is written in C++ and must be compiled prior to use.  The SDK is designed to be compiled by a standard C++ compiler installation; that is, no external libraries (beyond the standard and/or Posix libraries) are required.
The SDK may be built using either:
* Command line using CMAKE
* From Visual Studio Code
* From Visual Studio

To build the SDK outside Visual Studio, your platform must have the CMake tools installed.  To check if CMake is present, run:
```bash
cmake --version
```
### Building from the command line
Cmake can be invoked directly from the command line to build the SDK, as follows:
```
cd <IA SDK install path>/iasdk/build
cmake .
make
```

The build can be removed by running:
```
make clean
```

The build process will generate two executables by default:
```
/iasdk/build/testclient
/iasdk/build/navigationclient
```

## Building the C++17 SDK
For details on how to build the C++17 SDK, please review the `<IA SDK install path>/cpp17/README.md` file.

***Note - Colcon build will now automatically build the C++17 sdk components which are required for the ROS projects to function***

### Building from VS Code
The project is configured to work with the [Microsoft CMake Tools](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cmake-tools). 
If the CMake Tools are installed, opening the project in VSCode should detect the make files and configure the project accordingly.  The first time a build is selected you will need to select a compiler _kit_.  This will present a list of possible compiler configurations.


### Using the VS Code build task
The `.vscode` folder contains a build task configuration for invoking CMake.
To build:
* hit ***ctrl-shift-b***
* select ***Build***

(Note: you can also select ***Clean*** as one of the build options)

The build process will (again) generate two executables by default:
```
/iasdk/build/testclient
/iasdk/build/navigationclient
```


### Building from Visual Studio
If the Microsoft C++ compiler (msvc) is used, the SDK provides a Visual Studio solution (.sln) file.

Double-clicking on the .sln file will launch Visual Studio.
In Visual Studio select:

***Build -> Build Solution***

(Alternatively, use ***crtl-shift-b***)

## SDK API Overview

(TODO - add info on callbacks etc)

## C# Radar Client API

The .NET API is based on C#6 (.NET Framework 4.8) and was developed in Visual Studio 2019.
There are two project within the repro:

1. **IASDK** - The API DLL for use within any 3rd party projects to assist with connecting to the radar
1. **TestClient** - This is a very simple console application that runs up, connects to a radar and then displays some information before auto-disconnecting and closing. This provides a simple example of the recommended steps to connect and consume data from the radar.


### Usage of the SDK

The steps involved in connecting and getting data are as follows:

Setup your radar client and hook up the message and connection events:

```C#
_radarTcpClient = new RadarTcpClient();

_radarTcpClient.OnConfigurationData += ConfigurationDataHandler;

_radarTcpClient.OnFftData += FftDataHandler;

_radarTcpClient.OnConnectionChanged += ConnectionChangedHandler;
```

Connect to the radar:
```
_radarTcpClient.Connect("192.168.0.1");
```

On successful connection you will receive a Configuration message with details of the radar's current configuration. So you must have the handler setup before you connect.
```C#
private void ConfigurationDataHandler(object sender, GenericEventArgs<TcpConfigurationDataMessage> configurationMessage)
{
	var rotationHz = configurationMessage.Payload.RotationSpeed / 1000.0	
}
```

Once connected and you have the config data, tell the radar to start sending FFT Data:
```
_radarTcpClient.StartFftData();
```

You must handle incoming FFT Data:
```C#
private static void FftDataHandler(object sender, GenericEventArgs<FftData> fftEventArgs)
{
	var azimuth = fftEventArgs.Payload.Message.Azimuth;	
}
```

When you need to disconnect, firstly stop the FFT Data:
```
_radarTcpClient.StopFftData();
```

Then disconnect:
```
_radarTcpClient.Disconnect();
```

### Using the SDK in C++
The SDK C++ API provides an object-based interface for controlling/configuring the radar and for processing incoming data.

Communication to the radar is asynchronous:
* Outgoing messages are encapsulated within API calls.  The parameters provided to the functions are used to populate Colossus messages, which are sent to the sensor.
* Incoming messages from the sensor invoke callbacks.  Each (supported) incoming message will have a callback function object associated with it.  The callback is passed a (shared) pointer to the received data.

The API handles all endianness and encoding required by the Colossus protocol; removing the need from the client.


#### Connecting to the sensor
The steps involved in connecting and getting data are as follows:

Setup your radar client and hook up the message and connection events:
```
#include "radar_client.h"

int main()
{
    Navtech::Radar_client radar_client { "127.0.0.1"_ipv4 };

    // See below for details on message handler callbacks
    //
    radar_client.set_configuration_data_callback(config_handler);
	radar_client.set_fft_data_callback(fft_handler);

    ...
}
```

Connect to the radar:
```
int main()
{
    Navtech::Radar_client radar_client { "127.0.0.1"_ipv4 };

    radar_client.set_configuration_data_callback(config_handler);
	radar_client.set_fft_data_callback(fft_handler);

    // Connect to the radar
    //
    radar_client.start();
}
```

On successful connection you will receive a Configuration message with details of the radar's current configuration. So you must have the handler setup before you connect.


#### Message handler callbacks
An incoming Colossus message will be unmarshalled and stored in a message-specific structure.

Each structure also defines a pointer type (usually a `std::shared_ptr`).  This pointer type is used to allocate, and then access, the structure object created from the incoming Colossus message.

The message handler callback must be a _Callable Type_ - a function, a member-function, a function-object or a lambda expression.  

The signature of the _Callable Type_ for a data item of type _`Message_Ty`_ depends on the type of the Colossus message being received; for example:
```
void (*callback_fn_ptr)(                                      // Configuration data
    const Configuration_data::Pointer&,
    const Configuration_data::ProtobufPointer&
);

void (*callback_fn_ptr)(const Fft_data::Pointer&);            // FFT data

void (*callback_fn_ptr)(const std::vector<std::uint8_t>&);    // Raw FFT data      
```
Where `Pointer` is the message structure-specific pointer type.

For example:
```
void config_handler(
    const Configuration_data::Pointer& data, 
    const Configuration_data::ProtobufPointer& protobuf_config
)
{
    // Process incoming configuration...
}
```

Once connected and you have the config data, tell the radar to start sending FFT Data:
```
radar_client.start_fft_data();
```

You must provide a incoming FFT Data:
```
void fft_handler(const Fft_data::Pointer& data)
{
    // Process incoming FFT data...
}
```

When you need to disconnect, firstly stop the FFT Data:
```
radar_client.stop();
```

Then unbind the data handlers:
```
radar_client.set_fft_data_callback();
radar_client.set_configuration_data_callback();
```

Then disconnect:
```
radar_client.stop();
```

The file `testclient_main.cpp` contains an example of basic configuration and FFT data processing operations.  It can be used as the basis for more complex applications.