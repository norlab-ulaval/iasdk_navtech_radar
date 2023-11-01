/*
Copyright 2016 Navtech Radar Limited
This file is part of iasdk which is released under The MIT License (MIT).
See file LICENSE.txt in project root or go to https://opensource.org/licenses/MIT 
for full license details.
*/

namespace Navtech.IASDK.Enums
{
    /// <summary>
    /// Reflects state of a connection
    /// </summary>
    public enum ConnectionState
    {
        /// <summary>
        /// The disconnected state
        /// </summary>
        Disconnected = 0,
        /// <summary>
        /// The connecting state
        /// </summary>
        Connecting = 1,
        /// <summary>
        /// The connected state
        /// </summary>
        Connected = 2,
        /// <summary>
        /// The client has successfully registered
        /// </summary>
        Registered = 3,
        /// <summary>
        /// Disconnected as result of an error
        /// </summary>
        Faulted = 4
    }
}
