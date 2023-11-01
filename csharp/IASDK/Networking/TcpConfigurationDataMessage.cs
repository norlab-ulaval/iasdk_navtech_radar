using System;
using System.Runtime.InteropServices;
using Navtech.IASDK.Interfaces;

namespace Navtech.IASDK.Networking
{
    /// <summary>
    /// Represents a radar configuration data message
    /// </summary>
    [StructLayout(LayoutKind.Sequential, Size = 20, Pack = 1)]
    public struct TcpConfigurationDataMessage : IRawData
    {
        private readonly ushort _azimuthSamples;
        private readonly ushort _binSize;
        private readonly ushort _rangeInBins;
        private readonly ushort _encoderSize;
        private readonly ushort _rotationSpeed;
        private readonly ushort _packetRate;
        private readonly uint _rangeGain;
        private readonly uint _rangeOffset;

        /// <summary>
        /// Fixed size of the message
        /// </summary>
        public static int Size => 20;

        /// <summary>
        /// Number of azimuth samples the radar will take in a single rotation
        /// Typically 400
        /// </summary>
        public ushort AzimuthSamples => Utility.SwapUInt16(_azimuthSamples);

        /// <summary>
        /// The resolution size of each bin (in range)
        /// This is provided in millmetres
        /// </summary>
        public ushort BinSize => Utility.SwapUInt16(_binSize);

        /// <summary>
        /// Total number of bins for each sample
        /// </summary>
        public ushort RangeInBins => Utility.SwapUInt16(_rangeInBins);

        /// <summary>
        /// The size of the encoder wheel in the radar
        /// This can be used to convert the azimuth reading into degrees
        /// Typically enocder wheel is 5600
        /// </summary>
        public ushort EncoderSize => Utility.SwapUInt16(_encoderSize);

        /// <summary>
        /// The rotation speed of the radar
        /// This is provided in milli-Hertz
        /// </summary>
        public ushort RotationSpeed => Utility.SwapUInt16(_rotationSpeed);

        /// <summary>
        /// The expected number of packets per second being sent
        /// across the network from the radar
        /// </summary>
        public ushort PacketRate => Utility.SwapUInt16(_packetRate);

        /// <summary>
        /// The current range gain setting
        /// </summary>
        public float RangeGain => BitConverter.ToSingle(BitConverter.GetBytes(Utility.SwapUInt32(_rangeGain)), 0);

        /// <summary>
        /// The current range offset setting
        /// </summary>
        public float RangeOffset => BitConverter.ToSingle(BitConverter.GetBytes(Utility.SwapUInt32(_rangeOffset)), 0);

        /// <summary>
        /// Property that encodes all properties
        /// to provide a single byte array for the complete message
        /// </summary>
        public byte[] Data
        {
            get
            {
                var data = new byte[Size];
                var value = BitConverter.GetBytes(_azimuthSamples);
                Buffer.BlockCopy(value, 0, data, 0, 2);
                value = BitConverter.GetBytes(_binSize);
                Buffer.BlockCopy(value, 0, data, 2, 2);
                value = BitConverter.GetBytes(_rangeInBins);
                Buffer.BlockCopy(value, 0, data, 4, 2);
                value = BitConverter.GetBytes(_encoderSize);
                Buffer.BlockCopy(value, 0, data, 6, 2);
                value = BitConverter.GetBytes(_rotationSpeed);
                Buffer.BlockCopy(value, 0, data, 8, 2);
                value = BitConverter.GetBytes(_packetRate);
                Buffer.BlockCopy(value, 0, data, 10, 2);
                value = BitConverter.GetBytes(_rangeGain);
                Buffer.BlockCopy(value, 0, data, 12, 4);
                value = BitConverter.GetBytes(_rangeOffset);
                Buffer.BlockCopy(value, 0, data, 16, 4);
                return data;
            }
        }
    }
}