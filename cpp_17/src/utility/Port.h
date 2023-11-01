// ---------------------------------------------------------------------------------------------------------------------
// Copyright 2023 Navtech Radar Limited
// This file is part of IASDK which is released under The MIT License (MIT).
// See file LICENSE.txt in project root or go to https://opensource.org/licenses/MIT
// for full license details.
//
// Disclaimer:
// Navtech Radar is furnishing this item "as is". Navtech Radar does not provide 
// any warranty of the item whatsoever, whether express, implied, or statutory,
// including, but not limited to, any warranty of merchantability or fitness
// for a particular purpose or any warranty that the contents of the item will
// be error-free.
// In no respect shall Navtech Radar incur any liability for any damages, including,
// but limited to, direct, indirect, special, or consequential damages arising
// out of, resulting from, or any way connected to the use of the item, whether
// or not based upon warranty, contract, tort, or otherwise; whether or not
// injury was sustained by persons or property or otherwise; and whether or not
// loss was sustained from, or arose out of, the results of, the item, or any
// services that may be provided by Navtech Radar.
// ---------------------------------------------------------------------------------------------------------------------

#ifndef PORT_H
#define PORT_H


#include <functional>
#include <string>


namespace Navtech::Utility {

    // ----------------------------------------------------------------------------
    // A Port represents a 'point-of-interaction' between a composite class
    // and its environment.  That is, a Port provides a de-coupled mechanism for
    // getting data into/out of a composite object.
    // Ports are presented as public objects in the interface of the composite class.
    // 
    // If Port objects are connected together (using the forward_to() method) any data
    // posted to the Port will be forwarded-on to its connected companion.
    //
    // A function-object can be attached to a Port to process incoming data. If a Port
    // has a function-object registered, this will supercede forwarding the data on to
    // any connected port.
    // Normally, a function-object is used to pass arriving data in to the owning
    // composite object.
    //
    //     post(data)                                           on_receive([]{obj->fn(data);})
    // ______________              _______________              ____________________
    // |            |  forward_to  |  forward_to  |  forward_to |      ___________ |                 
    // |       out [ ] ---------> [ ] ---------> [ ] --------> [ ] --> |  obj    | |
    // |            |              | in       out |             | in   |         | |
    // |            |              |              |             |      ----------- |
    // --------------              ----------------             --------------------
    //
    template <typename T>
    class Port {
    public:
        using Callback = std::function<void(T&)>;
        using Value    = T;

        void forward_to(Port& dst)
        {
            other = &dst;
        }


        void disconnect()
        {
            other = nullptr;
        }


        template <typename U>
        void post(U&& in)
        {
            // Forward to connected port
            //
            if (other) {
                other->post(std::forward<U>(in));
            }
            // Hold locally
            //
            else {                            
                if (recv_cb) {
                    recv_cb(in);
                }
            }
        }


        template <typename U>
        void operator<<(U&& in)
        {
            post(std::forward<U>(in));
        }
        

        void on_receive(Callback fn)
        {
            recv_cb = std::move(fn);
        }

    private:
        Port*    other   { };
        Callback recv_cb { };
    };

} // namespace Navtech::Utility


#endif // PORT_H