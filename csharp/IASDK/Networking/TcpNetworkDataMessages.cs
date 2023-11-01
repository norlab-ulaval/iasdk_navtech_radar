/*
Copyright 2016 Navtech Radar Limited
This file is part of iasdk which is released under The MIT License (MIT).
See file LICENSE.txt in project root or go to https://opensource.org/licenses/MIT 
for full license details.
*/

using Navtech.IASDK.Interfaces;
using System;
using System.Runtime.InteropServices;

namespace Navtech.IASDK.Networking
{
    /// <summary>
    /// Represents an FFT network data message
    /// </summary>
    [StructLayout(LayoutKind.Sequential, Size = 14, Pack = 1)]
    public struct TcpFftDataMessage : IRawData
    {
        private readonly ushort _fftDataOffset;
        private readonly ushort _sweepCounter;
        private readonly ushort _azimuth;
        private readonly uint _seconds;
        private readonly uint _splitSeconds;

        /// <summary>
        /// Fixed 
        /// </summary>
        public static int Size => 14;

        /// <summary>
        /// Specifies where the FFT Data bytes start within the full message byte stream
        /// This allows for variation in the header size without changing the protocol
        /// </summary>
        public ushort FftDataOffset => Utility.SwapUInt16(_fftDataOffset);
        /// <summary>
        /// An incremental counter that is incremented by 1 for each azimuth processed
        /// </summary>
        public ushort SweepCounter => Utility.SwapUInt16(_sweepCounter);
        /// <summary>
        /// The azimuth that the FFT data in this message represents.
        /// This is reported as encoder counts. You will need the total encoder count
        /// value to convert this azimuth to a bearing. This is provided in the radar
        /// configuration message
        /// </summary>
        public ushort Azimuth => Utility.SwapUInt16(_azimuth);
        /// <summary>
        /// Seconds part of the time stamp
        /// </summary>
        public uint Seconds => Utility.SwapUInt32(_seconds);
        /// <summary>
        /// Split seconds part of the time stamp
        /// </summary>
        public uint SplitSeconds => Utility.SwapUInt32(_splitSeconds);

        /// <summary>
        /// Default constructor
        /// </summary>
        /// <param name="sweepCounter">Sweep count value</param>
        /// <param name="azimuth">Azimuth being processed</param>
        /// <param name="seconds">Seconds part of the time stamp</param>
        /// <param name="splitSeconds">Split seconds part of the time stamp</param>
        public TcpFftDataMessage(ushort sweepCounter, ushort azimuth, uint seconds, uint splitSeconds)
        {
            _fftDataOffset = Utility.SwapUInt16((ushort)Size);
            _sweepCounter = Utility.SwapUInt16(sweepCounter);
            _azimuth = Utility.SwapUInt16(azimuth);
            _seconds = Utility.SwapUInt32(seconds);
            _splitSeconds = Utility.SwapUInt32(splitSeconds);
        }

        /// <summary>
        /// Property that encodes all properties
        /// to provide a single byte array for the complete message
        /// </summary>
        public byte[] Data
        {
            get
            {
                var data = new byte[Size];
                var value = BitConverter.GetBytes(_fftDataOffset);
                Buffer.BlockCopy(value, 0, data, 0, 2);
                value = BitConverter.GetBytes(_sweepCounter);
                Buffer.BlockCopy(value, 0, data, 2, 2);
                value = BitConverter.GetBytes(_azimuth);
                Buffer.BlockCopy(value, 0, data, 4, 2);
                value = BitConverter.GetBytes(_seconds);
                Buffer.BlockCopy(value, 0, data, 6, 4);
                value = BitConverter.GetBytes(_splitSeconds);
                Buffer.BlockCopy(value, 0, data, 10, 4);
                return data;
            }
        }
    }
}