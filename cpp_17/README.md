# C++17 Implementation of the IASDK

This folder contains a C++17 standard version of the IASDK

This version of the SDK was developed on Ubuntu 20.04. 
The Preferred compiler for bulding is Clang V10 but GCC 9.3.x should work.


--------------------------------------------------------------------------------------------------------------------
## Building the SDK
### Prerequisites
```shell
sudo apt install build-essential clang g++ protobuf-compiler libprotobuf-dev cmake
```

### Building using CMAKE

This assumes that the SDK has been cloned into ~/iasdk.
To simplify the (arcane) CMake syntax, a simple batch script has been written to automate the build process.  The script creates a folder, `output`, which holds the build.  The script will create a subfolder - `debug`/`release` - depending on the option chosen at the commend line.

```shell
./bootstrap.sh [-v | --verbose] [ reset | clean | debug | release ]
```
Unless otherwise specified, the default build type is `debug`.

The output executables are placed in thr directory `bin`, below the build-type folder.  For example:
```
/iasdk/output/Debug/bin/colossus_client
/iasdk/output/Debug/bin/navigation_client
/iasdk/output/Debug/bin/connection_tester
/iasdk/output/Debug/bin/cat240_client
/iasdk/output/Debug/bin/nmea_server
/iasdk/output/Debug/bin/nmea_client
```

### Building from VS Code
The project is configured to work with the [Microsoft CMake Tools](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cmake-tools). 
If the CMake Tools are installed, opening the project in VSCode should detect the make files and configure the project accordingly.  The first time a build is selected you will need to select a compiler _kit_.  This will present a list of possible compiler configurations.


### Using the VS Code build task
The `.vscode` folder contains a build task configuration for invoking CMake.
To build:
* hit ***ctrl-shift-b***
* select ***Build***

The following options are available:
```
Make        Compile the debug configuration of the project (all executables)
Clean       Clean the project (next build forces a re-compile of all files)
Configure   Re-run CMake on the project; no build
Build       Configure, then make the debug configuration (same as Configure + Make)
```

--------------------------------------------------------------------------------------------------------------------
## Example projects
The SDK comes with three example projects.

### colossus_client
This project gives an example of how to create a radar client and connect up free-function handlers for processing various message types.
In this case, the radar client requests, and receives, FFT data from the radar.  The FFT data itself is not processed, but statistics about the data are presented - packet rate, packet size and packet time.

Note, when a client connects to a radar a configuration message is always sent.  The radar client must process this message, before requesting any other types.

The `colossus_client` program has two command-line options, as follows:
```
-i - The IP address of the server   [default: 127.0.0.1] 
-p - The server port                [default: 6317]
```
If an option is not provided, its default value will be used.

### navigation_client
The navigation_client project is a sample application that will peak search and report back upto ten targets per azimuth. 

The class Peak_finder can be used to process FFT data and search for peaks.
The algorithm will sub-resolve within radar bins and return a power at a distance in metres on the azmith being checked.

The algorithm implemented here will slide a window over the FFT data moving forwards by the size of the window, when the FFT has risen and then fallen, the peak resolving algorithm is run to sub-resolve the distance.

See Peak_finder.h for the data structure that is generated per azimuth

* threshold - Threshold in dB
* bins_to_operate_on - Radar bins window size to search for peaks in
* start_bin - Start Bin
* buffer_mode - Buffer mode should only be used with a staring radar
* buffer_length - Buffer length
* max_peaks_per_azimuth - Maximum number of peaks to find in a single azimuth

The `navigation_client` program has two command-line options, as follows:
```
-i - The IP address of the server   [default: 127.0.0.1] 
-p - The server port                [default: 6317] 
```
If an option is not provided, its default value will be used.

### Connection_tester
The Connection tester project is a utility to stress-test the stability of the server.  The program creates a radar client that randomly connects and disconnects from a server.  On each connection, the radar client requests
FFT data.

The `Connection_tester` object runs in its own thread-of-control and can therefore be run in parallel.  Up to three `Connection_tester` objects can be run, all connecting/disconnecting to the same radar.  More than three `Connection_tester` objects can be running, but a server will only allow a maximum of three connections at any one time.

 The `connection_tester` program has three command-line options, as follows:
```
-i - The IP address of the server             [default: 127.0.0.1] 
-p - The server port                          [default: 6317] 
-c - Number of connections to run in parallel [default: 1] 
```
If an option is not provided, its default value will be used.

NOTE:
The connection tester can be terminated with ctrl-c.  This will send an asynchronous message to each `Connection_tester` object.  The `Connection_tester` will only act upon this signal once it has completed its current connect/disconnect sequence.  This means it may take several seconds for the program to end, and you may see additional connect/disconnects after the ctrl-c.


### cat240_client
The `cat240_client` can connect to an ASTERIX CAT-240 source (for example, the `cat240_server` project) and will receive video messages.
This project does not perform any significant processing on incoming video messages.
NOTE:
The `cat240_client` will not (currently) re-form video message packets that may have been split (by the server).

The `cat240_server` program has two command-line options:
```
-i - The IP address UDP packets are read from [default: 127.0.0.1] 
-p - The port UDP packets are read from       [default: 6317]
```

### nmea_client
This project gives a simple example for receiving NMEA messages over a UDP connection.
At present, only the following NMEA messages can be handled:

* GPGGA
* GPRMC
* GPHDT
* PASHR

The messaging classes are currently simplistic and only perform basic message processsing (validation, conversion to string/vector, etc.)
The `nmea_client` program has two command-line options:
```
-i - The IP address UDP packets are read from [default: 127.0.0.1] 
-p - The port UDP packets are read from       [default: 6317]
```


### nmea_server
This project gives a simple example for sending NMEA messages over a UDP connection.
At present, only the following NMEA messages can be handled:

* GPGGA
* GPRMC
* GPHDT
* PASHR

The messaging classes allow construction of NMEA messages from strings or from vectors of `std::uint8_t`.  There are currently no facilities for appending/removing message clauses.
The `nmea_server` program has two command-line options:
```
-i - The IP address UDP packets sent to       [default: 127.0.0.1] 
-p - The port UDP packets are sent to         [default: 6317]
```
If the IP address supplied falls into the range of multicast addresses (224.0.0.1 - 239.255.255.255) the server will be configured to multicast its NMEA messages.


### colossus_protocol_tester
The `colossus_protocol_tester` application tests the Colossus messaging protocol between the radar and a client.  It verifies that the radar is responding to commands and emitting data correctly.
The program runs through a sequence of test cases, and presents the results.  Some tests may run for several seconds (for example, if messages are sent at periodic intervals).

The `colossuse_protocol_tester` program has three command-line options:
```
-i - The IP address UDP packets are read from       [default: 127.0.0.1] 
-p - The port UDP packets are read from             [default: 6317]
-t - The test case (specified as the message enum)  [ default: all]
```

For example:
```
$ ./colossus_protocol_tester -i 192.168.0.1 -t fft_data
```

--------------------------------------------------------------------------------------------------------------------
## Messaging protocol types
The SDK contains a minimum viable set of network messages for communicating to/from a radar.
Currently the following protocols are supported: 
* Colossus,
* Cambridge Pixel
* ASTERIX CAT-240
* NMEA (limited capability).

All of the protocols follow a similar idiom for how they are sent, received and accessed.
The protocols are designed to be extensible.  Please see the `/network/protocol/README.md` file for a detailed overview of how the Colossus protocol code works, how it used, and how it can be extended.


--------------------------------------------------------------------------------------------------------------------
## Utilities
The SDK comes with a library of utilities classes and functions, designed to make programming easier.
The utilities are used throughout the code.  Notable examples include:

### Time utilites (`Time_utils.h`)
The time utilities library provides an alternative to the C++ `std::chrono` library.  The Time Utilities support both
real-time and monotonic clocks.  The main classes are:
* Durations -  A `Duration` represents a period of time.  Durations have nanosecond resolution
* Observations - An `Observation` represents a point in time; as a `Duration` from their clock's epoch.

Durations and Observations are designed to be compared manipulated in intuitive ways - for example, subtracting two
Observations will yield the Duration between them.  Durations and Observations also support expressive streaming options, 
meaning displaying/streaming time objects is simple.

### Active classes (`Active.h`)
The Active class provides a high-level abstraction from OS threading and support for asynchronous messaging.
See the `Active.h` header for more information on how to use the `Active` class.

### Object lifetime managerment (`pointer_types.h`)
The code uses aliasing directives to make object lifetime management - ownership - as explicit as possible.
Raw pointers for dynamically-allocated objects is discouraged.
There is a distinct separation between object-to-object association (the "uses-a" relationship) and lifetime management of an object. owner_of and shared_owner types must *never* be used for association.

Under the hood:
* `owner_of` is an alias of `std::unique_ptr`;
* `shared_owner` is an alias of `std::shared_ptr`;
* `association_to` wraps 'raw' pointers

Please see Utility/pointer_types.h for a more detailed explanation.

## Networking utilities
The SDK contains a library of types for supporting networking.  These include:
```
IP_address      A lightweight type for storing and manipulating IPv4 addresses
Port            A type alias for TCP port IDs
Endpoint        A simple structure for specifying IP address + port
net_conversion  A library of host- to network-endian conversions for common types
```

--------------------------------------------------------------------------------------------------------------------

# Programming with the SDK
Connection to a radar is handled through a `Radar_client` object.  A `Radar_client` provides two basic interfaces:

* Setting up callbacks for incoming Colossus messages
* Sending Colossus messages (to the radar)


## Constructing the `Radar_client`
The `Radar_client` must be constructed with the IP address and port number of the server:
```C++
#include "Colossus_client.h"
#include "IP_address.h"

using namespace Navtech::Networking;
using namespace Navtech::Networking::Colossus_protocol;
using namespace Navtech::Utility;

int main()
{
    Radar_client radar_client { "198.168.0.1"_ipv4, "6317" };

    // more...
}
```

In the above code we are using the user-defined literal `_ipv4` to construct an `IP_address` object from a string literal.


## Creating a callback
Incoming messages are dispatched to an appropriate callback, which must be provided by the client.  The callback can be any C++ _Callable type_ - that is, a function, a member function or a lambda expression - that satisfies the function signature:
```C++
void (*callback)(Navtech::Networking::Colossus_protocol::Radar_client&, Navtech::Networking::Colossus_protocol::Message&);
```

Note, the callback has two parameters, the (incoming) message, and a reference to the (calling) `Radar_client`.  This reference allows the callback to access the API of the `Radar_client` without resorting to static (global) `Radar_client` objects.  An example of how this may be exploited is shown below. 

In this simple example, we'll use a free function to process the incoming configuration.
```C++
#include "Colossus_client.h"
#include "IP_address.h"
#include "Colossus_protocol.h"

using namespace Navtech::Networking;
using namespace Navtech::Networking::Colossus_protocol;
using namespace Navtech::Utility;

// Define a callback to process incoming configuration messages
//
void process_config(Radar_client& radar_client, Colossus_protocol::Message& msg)
{
    // See later...
}


int main()
{
    Radar_client radar_client { "198.168.0.1"_ipv4, "6317" };

    // more...
}
```

We must register the handler with the `Radar_client` and associate it with a particular message type.
```C++
#include "Colossus_client.h"
#include "IP_address.h"
#include "Colossus_protocol.h"

using namespace Navtech::Networking;
using namespace Navtech::Networking::Colossus_protocol;
using namespace Navtech::Utility;

void process_config(Radar_client& radar_client, Colossus_protocol::Message& msg)
{
    // See later...
}


int main()
{
    Radar_client radar_client { "198.168.0.1"_ipv4, "6317" };
    
    // Register the callback
    //
    radar_client.set_handler(Colossus_protocol::Type::configuration, process_config);

    // more...
}
```

Next, we must set the `Radar_client` running, so that it can connect to the radar and begin processing messages.  The `Radar_client` runs in its own thread.  Callbacks are executed in the context of the `Radar_client` (more specifically, in the context of the `Radar_client`'s `Dispatcher` thread). It is your responsibility to protect against race conditions if your callback functions interact with other threads-of-control.

For this example, we will simply let the `Radar_client` run for a period of time, before stopping.
```C++
#include "Colossus_client.h"
#include "IP_address.h"
#include "Colossus_protocol.h"

uusing namespace Navtech::Networking;
using namespace Navtech::Networking::Colossus_protocol;
using namespace Navtech::Utility;
using namespace Navtech::Time;
using namespace Navtech::Time::Monotonic;

void process_config(Radar_client& radar_client, Colossus_protocol::Message& msg)
{
    // See later...
}


int main()
{
    Radar_client radar_client { "198.168.0.1"_ipv4, "6317" };
    
    radar_client.set_handler(Colossus_protocol::Type::configuration, process_config);

    // Start the Radar_client and let it run for a period of time.
    // Note the use of a user-defined literal for the sleep duration.
    //
    radar_client.start();
    sleep_for(30_sec);
    radar_client.stop();
}
```

## Handling incoming messages
The `Colossus_protocol::Message` type encapsulates a buffer with a basic interface for accessing data. For more details on the `Message` type, please read `/network/protocol/README.md`, which explains the concept behind the Colossus messaging classes.

To access the radar data in a message, the simplest way (and our recommended way) is to perform a memory overlay onto the message data. The `Message` class provides an interface for this.  Once the overlay has been done, the specific message's API can be accessed to read from the message buffer.  The API has been constructed to hide any endianess issues, scaling or conversion that may be required to get from 'raw' message bytes to usable radar information.
```C++
#include "Colossus_client.h"
#include "IP_address.h"
#include "Colossus_protocol.h"
#include "Log.h"

using namespace Navtech::Networking;
using namespace Navtech::Networking::Colossus_protocol;
using namespace Navtech::Utility;
using namespace Navtech::Time;
using namespace Navtech::Time::Monotonic;

void process_config(Radar_client& radar_client, Colossus_protocol::Message& msg)
{
    // Interpret the message data as a Configuration message
    //
    Colossus_protocol::Configuration* config = msg.view_as<Colossus_protocol::Configuration>();

    // Access the data via the pointer
    //
    stdout_log << "Azimuth samples [" << config->azimuth_samples() << "]" << endl;
    stdout_log << "Bin size        [" << config->bin_size()<< "]" << endl;
    stdout_log << "Range in bins   [" << config->range_in_bins()<< "]" << endl;
    stdout_log << "Encoder size    [" << config->encoder_size()<< "]" << endl;
    stdout_log << "Rotation rate   [" << config->rotation_speed()<< "]" << endl;
    stdout_log << "Range gain      [" << config->range_gain()<< "]" << endl;
    stdout_log << "Range offset    [" << config->range_offset()<< "]" << endl;

    // more...
}
```

In the example above we are using the logging utility, `stdout_log`.  This utility extends C++ console output, whilst maintaining a similar interface (all the IO manipulators available to `std::ostream` are also available to `stdout_log`, for example).

## Handling protocol buffer from a message
If the incoming message contains a protocol buffer, this data can be extracted into a protobuf object and then accessed through the normal protobuf API.
```C++
#include "Colossus_client.h"
#include "IP_address.h"
#include "Colossus_protocol.h"
#include "Log.h"
#include "configurationdata.pb.h"
#include "Protobuf_helpers.h"

// As previously...

void process_config(Radar_client& radar_client, Colossus_protocol::Message& msg)
{
    using Navtech::Protobuf::from_vector_into;
    using namespace Colossus;
    using namespace Navtech::Networking;

    Colossus_protocol::Configuration* config = msg.view_as<Colossus_protocol::Configuration>();

    stdout_log << "Azimuth samples [" << config->azimuth_samples() << "]" << endl;
    stdout_log << "Bin size        [" << config->bin_size()<< "]" << endl;
    stdout_log << "Range in bins   [" << config->range_in_bins()<< "]" << endl;
    stdout_log << "Encoder size    [" << config->encoder_size()<< "]" << endl;
    stdout_log << "Rotation rate   [" << config->rotation_speed()<< "]" << endl;
    stdout_log << "Range gain      [" << config->range_gain()<< "]" << endl;
    stdout_log << "Range offset    [" << config->range_offset()<< "]" << endl;

    // Overlay onto the message memory, then extract the raw message payload
    // data into std::optional protobuf object using the helper function
    // Protobuf::from_vector_into.  If the conversion fails, the function
    // will return std::nullopt
    //
    std::optional<Protobuf::ConfigurationData> protobuf = from_vector_into<Protobuf::ConfigurationData>(config->to_vector());

    if (protobuf.has_value()) {
        stdout_log << "Radar ID   [" << protobuf->model().id()   << "]" << endl;
        stdout_log << "Radar name [" << protobuf->model().name() << "]" << endl;
        // etc...
    }
}
```

## Sending a message
The full details of sending Colossus messages is beyond the scope of this 'getting started' overview.  For full details of how to construct and send Colossus messages, please read `/network/protocol/README.md`.

In the case of a radar client, typically only simple messages are sent to the radar, to enable/disable features such as FFT data, or radar health.

Remember - if you enable data transmission from the radar you *must* have a handler for it, otherwise you will receive an error each time a message arrives!
```C++
void process_FFT(Radar_client& radar_client, Colossus_protocol::Message& msg)
{
    // Process the incoming FFT data...
}


int main()
{
    Radar_client radar_client { "192.168.0.1"_ipv4, "6317" };
    
    radar_client.set_handler(Colossus_protocol::Type::configuration, process_config);
    radar_client.set_handler(Colossus_protocol::Type::fft_data, process_FFT);

    radar_client.start();
    sleep_for(5_sec);

    // Construct a message to start FFT data and send it to the radar
    //
    Colossus_protocol::Message msg { };
    msg.type(Colossus_protocol::Type::start_fft_data);

    radar_client.send(std::move(msg));

    sleep_for(30_sec);
    radar_client.stop();
}
```

## Sending a message within a callback
Commonly, you will want to send a message from within a callback. Since the callback has the `Radar_client` as an argument, you can directly send a message.

Note, in the case of simple messages (with no header or payload), the `send()` method can construct the `Colossus_protocol::Message` object implicitly as part of the call.
```C++
void process_config(Radar_client& radar_client, Colossus_protocol::Message& msg)
{
    using Navtech::Protobuf::from_vector_into;
    using namespace Colossus;
    using namespace Navtech::Networking;

    Colossus_protocol::Configuration* config = msg.view_as<Colossus_protocol::Configuration>();

    stdout_log << "Azimuth samples [" << config->azimuth_samples() << "]" << endl;
    stdout_log << "Bin size        [" << config->bin_size()<< "]" << endl;
    stdout_log << "Range in bins   [" << config->range_in_bins()<< "]" << endl;
    stdout_log << "Encoder size    [" << config->encoder_size()<< "]" << endl;
    stdout_log << "Rotation rate   [" << config->rotation_speed()<< "]" << endl;
    stdout_log << "Range gain      [" << config->range_gain()<< "]" << endl;
    stdout_log << "Range offset    [" << config->range_offset()<< "]" << endl;

    
   auto protobuf = from_vector_into<Protobuf::ConfigurationData>(config->to_vector());

    if (protobuf.has_value()) {
        stdout_log << "Radar ID   [" << protobuf->model().id()   << "]" << endl;
        stdout_log << "Radar name [" << protobuf->model().name() << "]" << endl;
    }

    // Send a simple message to the radar
    //
    radar_client.send(Colossus_protocol::Type::start_fft_data);
}
```

