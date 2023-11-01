/*
Copyright 2016 Navtech Radar Limited
This file is part of iasdk which is released under The MIT License (MIT).
See file LICENSE.txt in project root or go to https://opensource.org/licenses/MIT 
for full license details.
*/

namespace Navtech.IASDK.Interfaces
{
    /// <summary>
    /// Represemts the most basic type of raw message type
    /// </summary>
    public interface IRawData
    {
        /// <summary>
        /// Message Data
        /// </summary>
        byte[] Data { get; }
    }
}
