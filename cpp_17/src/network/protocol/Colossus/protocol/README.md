# COLOSSUS PROTOCOL API

# Introduction
## Definitions
The Colossus message protocol, as with many network communication protocols, distinguishes between _header_ information - that is, meta-data about the message, its type, size, etc., and _payload information_ - the application-specific data.  
However, with the Colossus protocol there is added confusion as messages can have two types of header, as well as an optional.  Confusingly, the terms header and payload may be used contextually (and sometimes interchangeably!).  To attempt to alleviate this, the following definitions will be used.

### TCP Message Header
Every Colossus message has a _TCP Message Header_.  The _TCP Message Header_ contains the message type, the overall message size, and validation information (version and signature).

### Colossus Message Header
The _Colossus Message Header_ contains fixed-format, application-specific data for a particular Colossus message.  The _Colossus Message Header_ is optional.
If a message contains a _Colossus Message Header_ there will be an API to access its elements.

### Colossus Message Payload
A Colossus message may, optionally, contain a variable-length data block that must be interpreted by the receiving application.  Commonly, the _Colossus Message Payload_ is in the form of a Google Protocol Buffer.

A Colossus message may, therefore, be in one of four formats:
* Just a _TCP Message Header_ (sometimes referred to as a _Simple_ message)
* A _TCP Message Header_ and _Colossus Message Header_
* A _TCP Message Header_, _Colossus Message Header_ and _Colossus Message Payload_
* A _TCP Message Header_ _Colossus Message Payload_

The memory layout of these messages is shown below
```
______________________
| TCP Message Header | 
----------------------

________________________________________________
| TCP Message Header | Colossus Message Header |
------------------------------------------------

_____________________________________________________________________________
| TCP Message Header | Colossus Message Header | Colossus Message Payload   |
-----------------------------------------------------------------------------

___________________________________________________
| TCP Message Header | Colossus Message Payload   |
---------------------------------------------------

```
## Colossus Message classes
The following classes paricipate in the Colossus protocol.

### `Networking::Colossus_protocol::Message`
The `Message` class acts as the primary storage type for Colossus messages.  Data is stored as a dynamically-allocated contiguous sequence.

The `Message` class provides the _TCP Message Header_ as part of its default construction.

The `Message` class provides interfaces for:
* Accessing/modifying _TCP Message Header_ data
* Accessing overall message size 
* Appending _Colossus Message Header_ and _Colossus Message Payload_ data.  
* Retreiving the message data as a buffer of bytes (for transmission).

NOTE: The `Message` class provides no interface for modifying _Colossus Message Header_ or _Colossus Message Payload_ data.

### `Networking::Colossus_protocol::Message_base::Simple`
The `Simple` class (alias) is used as a base class for messages that consist of a _TCP Message Header_ only.

### `Networking::Colossus_protocol::Message_base::Header_only`
The `Header_only` class is used as a base class for Colossus messages that have a _Colossus Message Header_-only structure.

### `Networking::Colossus_protocol::Message_base::Header_and_payload`
The `Header_and_payload` class is used as a base class for Colossus messages that have both a _Colossus Message Header_ and _Colossus Message Payload_.

### `Networking::Colossus_protocol::Message_base::Payload_only`
The `Header_only` class (alias) is used as a base class for Colossus messages that have a _Colossus Message Payload_-only structure.

### Colossus messages (`colossus_messages.h`)
The Colossus message classes represent individual message types in the Colossus protocol.
The Colossus messages are used in two ways:
* As a mechanism to _interpret_ the data in a `Colossus_protocol::Message`; for example, when the data is received over a network connection.
* As an interface for intialisation the fixed-structure aspects of the _Colossus Message Header_, prior to being appended to a `Message`

Most common Colossus messages will be pre-defined by the SDK.  However, new message types can be added.  See _Creating new Colossus messages_ (below) for more information.

# Creating a Message from an incoming data stream
This section looks at how to use the Colossus message classes to store, validate and access an incoming Colossus message stream; for example, over a TCP network connection.

## Constructing a message from a data stream
A typical approach, when receiving variable-length messages over a network, is to:
* Read a header's worth of data
* Validate the header
* Extract the payload size from the header
* Read the payload data from the network
* Process the resultant (complete) message

The `Networking::Colossus_protocol::Message` class provides an interface for such operations.
```
using namespace Navtech::Networking;

std::vector<std::uint8_t> header_buffer { };
std::vector<std::uint8_t> payload_buffer { };

Colossus_protocol::Message msg { };

auto header_sz = msg.header_size();


// Read header_sz bytes from the network into header_buffer...

// The default-constructed Message provides a TCP Message Header,
// so when the new header arrives the existing header must be
// replaced
//
msg.replace(header_buffer);


auto payload_sz = msg.payload_size();

if (payload_sz > 0) {

    // Read payload_sz bytes from the network 
    // into payload_buffer...

    // Payload data must be appended to the end 
    // of the message
    //
    msg.append(payload_buffer);
}
```


## Accessing data via the Message's API
The Colossus message classes defined in `colossus_messages.h` provide an interface for interpreting and accessing Colossus message data in a `Message` class.

When accessing a populated `Message` object these class are used as an _overlay_ onto the `Message`'s internal data.

For the purposes of this example we'll take the most-complex case of a Colossus message that has both a _Colossus Message Header_ and _Colossus Message Payload_; with the caveats that:
* A message without a _Colossus Message Payload_ will not have not provide the API for these features
* A message without a _Colossus Message Header_ will not have accessor/mutator methods for manipulating any of the header's elements

To access the message data the `view_as` method is used.  This template function acts like a `reinterpret_cast` that returns a pointer to the 'raw' message data, but interpreted as a Colossus message class type.

```
using namespace Navtech::Networking;

void process_message(const Colossus_protocol::Message& msg)
{
    auto fft_view = msg.view_as<Colossus_protocol::Fft_data>();
    ...
}
```


The message overlay can be visualised as follows:
```
msg
|
______________________
| TCP Message Header | 
----------------------
|                    |
_____________________________________________________________________________
|                           RAW MESSAGE DATA                                |
-----------------------------------------------------------------------------
                     ^                         ^                            ^
                     |                         |                            |
                     ________________________________________________________
Header_and_payload   | Colossus Message Header | Colossus Message Payload   |
                     --------------------------------------------------------
                     ^                         ^
                     |                         |
                     ---------------------------
Fft_data:            | os | sc | az | sec | ss |
                     ---------------------------
                     ^
                     |
                     fft_view*

```
The base class `Message_base::Header_and_payload` provides appropriate offsets into the `Message`'s data stream.  The derived `Fft_data` class provides the fixed-sized overlay onto the _Colossus Message Header_.

_(Notice, internally, the `Message` class holds an overlay to the _TCP Message Header_ and uses that to access, for example, payload size or message type)_

The view-pointer can now be used to access the Fft_data class's API for accessing _Colossus Messsage Header_ fields.  Using the interface ensures that (for example) value validation and endianness correction is performed.

**Note - if the message has no _Colossus Message Header_ there will be no API functions defined.**

```
using namespace Navtech::Networking;

void process_message(const Colossus_protocol::Message& msg)
{
    auto fft_view = msg.view_as<Colossus_protocol::Fft_data>();
    
    auto azimuth           = fft_view->azimuth();
    auto angle             = fft_view->azimuth() * 360.0F;
    auto sweep_counter     = fft_view->sweep_counter();
    auto ntp_seconds       = fft_view->ntp_seconds();
    auto ntp_split_seconds = fft_view->ntp_split_seconds();
    ...
}
```

To access the (variable-length) _Colossus Message Payload_ two helper methods are provided - `to_string` and `to_vector`.  The `to_string` method is often useful if the _Colossus Message Payload_ is in the form of a protocol buffer.


**Note - if the message does not have a _Colossus Message Payload_ these methods will not be available.**


```
using namespace Navtech::Networking;

void process_message(const Colossus_protocol::Message& msg)
{
    auto fft_view = msg.view_as<Colossus_protocol::Fft_data>();
    
    std::uint16_t azimuth              { fft_view->azimuth() };
    std::uint16_t sweep_counter        { fft_view->sweep_counter() };
    std::uint32_t ntp_seconds          { fft_view->ntp_seconds() };
    std::uint32_t ntp_split_seconds    { fft_view->ntp_split_seconds() };
    
    std::vector<std::uint8_t> fft_data { fft_view->to_vector() };
}
```

# Sending a Message
In this section we'll look at the second use of the Colossus message types: to create a message to send over the network. For this example, the focus will be on the construction of the message, rather than on the transmission of the message.

Again, we'll use the most-complex message type: a message with both a _Colossus Message Header_ and a _Colossus Message Payload_.  For messages without one of these components, simply omit the relevant steps.

There are three stages to constructing a message to send:
* Creating the _Colossus Message Header_ object
* Appending the _Colossus Message Payload_ data
* Retrieving the 'raw' data buffer for sending

For example:
```
using namespace Navtech::Networking;

void send_FFT_data(std::uint16_t azi, std::vector<std::uint8_t>&& fft_data)
{
    // Note: We are not using the Fft_data object as
    // an overlay, rather as a 'normal' object
    //
    Colossus_protocol::Fft_data header { };

    header.azimuth(azi);
    header.sweep_counter(++count);

    std::timespec now { };
    std::timespec_get(&now, TIME_UTC);
    header.ntp_seconds(now.tv_sec);
    header.ntp_subseconds(now.tv_nsec);

    Colossus_protocol::Message msg { };

    msg.type(Colossus_protocol::Message::Type::fft_data);
    msg.append(std::move(header));
    msg.append(std::move(fft_data));

    <network_send_function>(msg.relinquish());
}
```
Notes on the above code:
* The `Message` supports insertion by both move and copy.
* The header and payload can be appended in any order; the Message class ensures the header is always located before the payload.  for efficiency, it is always recommended the header is appended before the payload.
* The Message class supports stream operators; for example: `msg << header << fft_data;` is supported.
* The `relinquish()` method returns a `std::vector<std::uint8_t>` and empties the `Message`; as if the `Message` has been _moved-from_.
* `network_send_function` is a generic placeholder name for whatever your network send function is.



# Creating new Colossus messages
Common Colossus message types are provided in the SDK.  It may, however, sometimes be necessary to add new messages to the protocol.

This section provides details for how to extended the protocol.  

For completeness we will consider a message that has both a _Colossus Message Header_ and _Colossus Message Payload_.

## Message base types
Any new message must be derived from one of the Colossus protocol base types.  For convenience, these are repeated below:

### `Networking::Colossus_protocol::Message_base::Simple`
The `Simple` class (alias) is used as a base class for messages that consist of a _TCP Message Header_ only.

### `Networking::Colossus_protocol::Message_base::Header_only`
The `Header_only` class is used as a base class for Colossus messages that have a _Colossus Message Header_-only structure.

### `Networking::Colossus_protocol::Message_base::Header_and_payload`
The `Header_and_payload` class is used as a base class for Colossus messages that have both a _Colossus Message Header_ and _Colossus Message Payload_.

### `Networking::Colossus_protocol::Message_base::Payload_only`
The `Header_only` class (alias) is used as a base class for Colossus messages that have a _Colossus Message Payload_-only structure.

Note - these classes are templates, where the template parameter is the name of the derived type (that is, the message (header) class you are defining).  This is an application of the Curiously Recurring Template Pattern, used here to ensure that overlay alignment of the new message is correct (also, see below for more).

## Defining a message
Only the header information needs to be defined for the new message; all other requirements are handled by the base class.

Notice, the new message **MUST** be packed to a byte boundary as it will be used as an overlay on a contiguous byte array.

_(If new messages are being defined in `colossus_messages.h` (which is the preferred option) the `#pragma pack()` directives are included at the top and bottom of the file; and so cover all messages)_

```
// colossus_messages.h
//

namespace Navtech::Networking::Colossus_protocol {

    // Pack to the nearest byte boundary
    //
    #pragma pack(1)

    class Navigation_data : public Message_base::Header_and_payload<Navigation_data> {
    public:
        // More to follow...

    private:
        // More to follow...
    };


    #pragma pack()
} // namespace Navtech::Networking::Colossus_protocol
```

The attributes of the header must be of the same size, and declared in the same order, as the fields of the _Colossus Message Header_.  The declaration order is from low address to high address.

```
namespace Navtech::Networking::Colossus_protocol {

    #pragma pack(1)

    class Navigation_data : public Message_base::Header_and_payload<Navigation_data> {
    public:
        // More to follow...

    private:
        // Low address
        //
        std::uint16_t net_azimuth;
        std::uint32_t seconds;
        std::uint32_t split_seconds;
        //
        // High address
    };


    #pragma pack()
} // namespace Navtech::Networking::Colossus_protocol
```

Accessor/mutator ('getter'/'setter') methods should be added for each attribute.  These functions must ensure that attributes are stored in network-endian order. To assist you, the utility library `net_conversion.h` has been provided. (Note, the conversion functions are all in the namespace `Navtech::Utility`)

These methods can also be used to perform any invariant-checking required to maintain message header validity.  

```
namespace Navtech::Networking::Colossus_protocol {

    using namespace Utility;

    #pragma pack(1)

    class Navigation_data : public Message_base::Header_and_payload<Navigation_data> {
    public:
        std::uint16_t azimuth() const 
        {
            return from_uint16_network(net_azimuth); 
        }
        
        void azimuth(std::uint16_t val) 
        { 
            net_azimuth = to_uint16_network(val); 
        }

        std::uint32_t ntp_seconds() const 
        { 
            return from_uint32_network(seconds); 
        }
        
        void ntp_seconds(std::uint32_t val) 
        { 
            seconds = to_uint32_network(val); 
        }

        std::uint32_t ntp_split_seconds() const 
        { 
            return from_uint32_network(split_seconds); 
        }

        void ntp_split_seconds(std::uint32_t val) 
        { 
            split_seconds = to_uint32_network(val); 
        }

    private:
        std::uint16_t net_azimuth;
        std::uint32_t seconds;
        std::uint32_t split_seconds;
    };


    #pragma pack()
} // namespace Navtech::Networking::Colossus_protocol
```

Finally, the new message must provide a size() method.  This is used by the base class to correctly align the memory overlay.

**Note - If this method returns the incorrect header size, or is not defined, the behaviour of the class is undefined.**

```
namespace Navtech::Networking::Colossus_protocol {
****
    using namespace Utility;

    #pragma pack(1)

    class Navigation_data : public Message_base::Header_and_payload<Navigation_data> {
    public:
        std::uint16_t azimuth() const 
        {
            return from_uint16_network(net_azimuth); 
        }
        
        void azimuth(std::uint16_t val) 
        { 
            net_azimuth = to_uint16_network(val); 
        }

        std::uint32_t ntp_seconds() const 
        { 
            return from_uint32_network(seconds); 
        }
        
        void ntp_seconds(std::uint32_t val) 
        { 
            seconds = to_uint32_network(val); 
        }

        std::uint32_t ntp_split_seconds() const 
        { 
            return from_uint32_network(split_seconds); 
        }

        void ntp_split_seconds(std::uint32_t val) 
        { 
            split_seconds = to_uint32_network(val); 
        }

        // If your message has a header you MUST 
        // provide this function.
        //
        std::size_t size() const 
        { 
            return (sizeof(std::uint16_t) + 2 * sizeof(std::uint32_t)); 
        }

    private:
        std::uint16_t net_azimuth;
        std::uint32_t seconds;
        std::uint32_t split_seconds;
    };


    #pragma pack()
} // namespace Navtech::Networking::Colossus_protocol
```


# Extended message API (for special cases)
The above API should be sufficient to perform the majority of Colossus protocol operations.  However, in some special circumstances (for example, as a consequence of low-level network handling) additional control may be required.  The Colossus message API has additional methods than can be exploited which may be useful.


## Signature
A `Signature` class encapsulates the Colossus message synchronisation sequence.  It provides a simple interface allowing it to be compared for equality/inequality; both with other `Signature` objects and (more usefully) with `std::vector<std::uint8_t>`.

The `Message` class has a static method `valid_signature()`, which returns an initialised `Signature` object.

## Iterators
Iterators provide 'raw' access to a `Message`'s data array.  These iterators can be used to extract data from the `Message`.

It is NOT recommend to attempt to insert into the `Message` data array via an iterator.

Iterators are provided by the `Message_base::Header_only`,  `Message_base::Header_and_payload` and `Message_base::Payload_only` classes, as follows

### Message_base::Header_only
* `begin()` points to the start of the _Colossus Message Header_ (or, if you prefer, to the start of the _TCP Message Payload_)
* `end()` points to one-past-the-end of the _Colossus Message Header_


### Message_base::Header_and_payload
* `begin()` points to the start of the _Colossus Message Header_ (or, if you prefer, to the start of the _TCP Message Payload_)
* `end()` points to one-past-the-end of the _Colossus Message Header_
* `payload_begin()` points to the start of the _Colossus Message Payload_ (that is, the same address as `end()`)
* `payload_end()` points to one-past-the-end of the _Colossus Message Payload_ 


### Message_base::Payload_only
* `payload_begin()` points to the start of the _Colossus Message Payload_ (that is, the same address as `end()`)
* `payload_end()` points to one-past-the-end of the _Colossus Message Payload_ 

The diagram below summarises this configuration:

```
______________________
| TCP Message Header | 
----------------------
|                    |
________________________________________________
|          RAW MESSAGE DATA                    |
------------------------------------------------
                     |                         |
                     ___________________________
Header_only          | Colossus Message Header | 
                     ---------------------------
                     ^                          ^
                     | begin()            end() |



___________________________________________________________________________
|                           RAW MESSAGE DATA                              |
---------------------------------------------------------------------------
                     |                         |                          |
                     ______________________________________________________
Header_and_payload   | Colossus Message Header | Colossus Message Payload |
                     ------------------------------------------------------
                     ^                          ^                          ^
                     | begin()            end() |                          |
                                payload_begin() |            payload_end() |



_________________________________________________
|          RAW MESSAGE DATA                     |
-------------------------------------------------
                     |                          |
                     ____________________________
Payload_only         | Colossus Message Payload | 
                     ----------------------------
                     ^                           ^
                     | payload_begin()           | payload_end()