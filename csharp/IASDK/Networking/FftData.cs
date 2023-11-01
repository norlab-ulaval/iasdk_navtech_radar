/*
Copyright 2016 Navtech Radar Limited
This file is part of iasdk which is released under The MIT License (MIT).
See file LICENSE.txt in project root or go to https://opensource.org/licenses/MIT 
for full license details.
*/

using Navtech.IASDK.Interfaces;

namespace Navtech.IASDK.Networking
{
    /// <summary>
    /// Represents an FFT Data message and implements the <see cref="IRawData"/> interface
    /// Includes both the raw data and the class represents of the message
    /// </summary>
    public class FftData : IRawData
    {
        /// <summary>
        /// Represents network data as a TCP FFT Data Message <see cref="TcpFftDataMessage"/>
        /// </summary>
        public TcpFftDataMessage Message { get; private set; }

        /// <summary>
        /// The raw network message bytes 
        /// </summary>
        public byte[] Data { get; private set; }

        /// <summary>
        /// Default constructor
        /// </summary>
        /// <param name="message">TCP FFT Message</param>
        /// <param name="data">Message as raw bytes</param>
        public FftData(TcpFftDataMessage message, byte[] data)
        {
            Message = message;
            Data = data;
        }
    }
}
