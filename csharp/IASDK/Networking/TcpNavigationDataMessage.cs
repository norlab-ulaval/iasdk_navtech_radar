using Navtech.IASDK.Interfaces;
using System;
using System.Runtime.InteropServices;

namespace Navtech.IASDK.Networking
{
    /// <summary>
    /// Represents a network navigation data message
    /// </summary>
    [StructLayout(LayoutKind.Sequential, Size = 10, Pack = 1)]
    public struct TcpNavigationDataMessage : IRawData
    {
        private readonly ushort _azimuth;
        private readonly uint _seconds;
        private readonly uint _splitSeconds;

        /// <summary>
        /// Fixed message size
        /// </summary>
        public static int Size => 10;

        /// <summary>
        /// Current azimuth being reported
        /// </summary>
        public ushort Azimuth => Utility.SwapUInt16(_azimuth);
        /// <summary>
        /// Seconds part of time stamp
        /// </summary>
        public uint Seconds => Utility.SwapUInt32(_seconds);
        /// <summary>
        /// Split seconds part of the time stamp
        /// </summary>
        public uint SplitSeconds => Utility.SwapUInt32(_splitSeconds);

        /// <summary>
        /// Default constructor
        /// </summary>
        /// <param name="azimuth">Current azimuth</param>
        /// <param name="seconds">Seconds part of time stamp</param>
        /// <param name="splitSeconds">Split seconds part of the time stamp</param>
        public TcpNavigationDataMessage(ushort azimuth, uint seconds, uint splitSeconds)
        {
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
                var value = BitConverter.GetBytes(_azimuth);
                Buffer.BlockCopy(value, 0, data, 0, 2);
                value = BitConverter.GetBytes(_seconds);
                Buffer.BlockCopy(value, 0, data, 2, 4);
                value = BitConverter.GetBytes(_splitSeconds);
                Buffer.BlockCopy(value, 0, data, 6, 4);
                return data;
            }
        }
    }
}