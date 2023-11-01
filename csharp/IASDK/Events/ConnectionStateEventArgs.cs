/*
Copyright 2016 Navtech Radar Limited
This file is part of iasdk which is released under The MIT License (MIT).
See file LICENSE.txt in project root or go to https://opensource.org/licenses/MIT 
for full license details.
*/

using Navtech.IASDK.Enums;
using System;

namespace Navtech.IASDK.Events
{
    /// <summary>
    /// EventArgs for sharing ConnectionState
    /// </summary>
    public class ConnectionStateEventArgs : EventArgs
    {
        /// <summary>
        /// The connection state
        /// </summary>
        public ConnectionState State { get; }

        /// <summary>
        /// Default Constructor
        /// </summary>
        /// <param name="state">Connection State</param>
        public ConnectionStateEventArgs(ConnectionState state)
        {
            State = state;
        }
    }
}
